"""Train an XGBoost snag classifier on a cable-position log.

Usage (from the repo root):
    python3 snag_detection/train.py --synthetic        # no labelled field data yet
    python3 snag_detection/train.py data/my_log.csv    # columns: t, x, y, snag

The log has one row per localization cycle (10 Hz) with columns ``t`` (s),
``x``, ``y`` (in, from any localization method) and ``snag`` (0/1). Features
are trailing-window statistics (see features.py), and the train/test split is
chronological because the rows are a time series: a shuffled split would put
near-identical neighbouring samples on both sides and inflate the scores.

The model is saved to snag_detection/models/snag_xgb.json. To use it:

    from xgboost import XGBClassifier
    from features import build_features
    clf = XGBClassifier()
    clf.load_model("snag_detection/models/snag_xgb.json")
    is_snag = clf.predict(build_features(df))   # one 0/1 per row
"""
import argparse
from pathlib import Path

import pandas as pd
from sklearn.metrics import classification_report, confusion_matrix
from xgboost import XGBClassifier

from features import FEATURE_NAMES, LONG_WINDOW, build_features

MODEL_PATH = Path(__file__).resolve().parent / "models" / "snag_xgb.json"


def train(df, test_frac=0.25, seed=0):
    X = build_features(df)
    y = df["snag"].astype(int)
    X, y = X.iloc[LONG_WINDOW:], y.iloc[LONG_WINDOW:]  # skip rolling-window warm-up

    split = int(len(X) * (1 - test_frac))
    X_tr, X_te, y_tr, y_te = X.iloc[:split], X.iloc[split:], y.iloc[:split], y.iloc[split:]

    pos = int(y_tr.sum())
    clf = XGBClassifier(
        n_estimators=300,
        max_depth=4,
        learning_rate=0.05,
        subsample=0.8,
        colsample_bytree=0.8,
        scale_pos_weight=(len(y_tr) - pos) / max(pos, 1),  # snags are rare
        eval_metric="logloss",
        random_state=seed,
    )
    clf.fit(X_tr, y_tr)
    return clf, X_te, y_te


def report(clf, X_te, y_te):
    pred = clf.predict(X_te)
    print(f"Test rows: {len(y_te)} (snag rows: {int(y_te.sum())})")
    print("Confusion matrix [[TN FP] [FN TP]]:")
    print(confusion_matrix(y_te, pred))
    print(classification_report(y_te, pred, target_names=["normal", "snag"], digits=3, zero_division=0))
    print("Feature importance:")
    for w, name in sorted(zip(clf.feature_importances_, FEATURE_NAMES), reverse=True):
        print(f"  {name:<16} {w:.3f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="?", help="labelled log with columns t, x, y, snag")
    ap.add_argument("--synthetic", action="store_true", help="train on simulated data from simulate.py")
    ap.add_argument("--out", type=Path, default=MODEL_PATH, help="where to save the model")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    if args.synthetic:
        from simulate import simulate

        df = simulate(seed=args.seed)
    elif args.csv:
        df = pd.read_csv(args.csv)
    else:
        ap.error("give a CSV path or --synthetic")

    clf, X_te, y_te = train(df, seed=args.seed)
    report(clf, X_te, y_te)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    clf.save_model(args.out)
    print(f"Saved model to {args.out}")


if __name__ == "__main__":
    main()
