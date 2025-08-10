import math
import openpyxl

def convert_excel_to_sensor_format(excel_file):
    """
    Converts LiDAR_Data_collection.xlsx data to the format used in sensordata.py
    Input data is in mm and degrees
    Output will be in inches and radians to match simulation format
    """
    # Load the workbook and select the active sheet
    wb = openpyxl.load_workbook(excel_file)
    sheet = wb.active
    
    # Convert mm to inches
    MM_TO_INCHES = 1/25.4
    
    # Initialize list to store all readings
    all_readings = []
    
    # Find the starting row of data (skip headers)
    start_row = 4  # Data starts on line 4

    # Column indices for each sensor's data (1-based indexing)
    sensor_columns = [
        (4, 5),   # Sensor 1: Angle (D), Distance (E) columns
        (9, 10),  # Sensor 2: Angle (I), Distance (J) columns
        (14, 15), # Sensor 3: Angle (N), Distance (O) columns
        (19, 20)  # Sensor 4: Angle (S), Distance (T) columns
    ]
    
    # Process each row of data
    row = start_row
    while True:
        has_valid_data = False
        
        # Process each sensor
        for angle_col, dist_col in sensor_columns:
            angle_cell = sheet.cell(row=row, column=angle_col).value
            dist_cell = sheet.cell(row=row, column=dist_col).value
            
            # Skip if we've reached the end of data
            if angle_cell is None or dist_cell is None:
                continue
                
            has_valid_data = True
            
            # Convert units if the reading is valid
            if dist_cell != 65535:  # Check for invalid LIDAR reading
                distance_inches = float(dist_cell) * MM_TO_INCHES
                angle_rad = math.radians(float(angle_cell))
                
                # Adjust angles based on the specified rules
                if sensor_columns.index((angle_col, dist_col)) == 0:
                    angle_rad -= math.pi *1.5

                if sensor_columns.index((angle_col, dist_col)) == 1:
                    angle_rad = -angle_rad + math.pi

                if sensor_columns.index((angle_col, dist_col)) == 2:
                    angle_rad -= 0#math.pi *2

                if sensor_columns.index((angle_col, dist_col)) == 3:
                    angle_rad -= 0.5 * math.pi
                    angle_rad = -angle_rad
                
                all_readings.append([distance_inches, angle_rad])
        
        # If no valid data in this row, we've reached the end
        if not has_valid_data:
            break
            
        row += 1
        
    
    return all_readings

def main():
    try:
        # Replace with your Excel file path
        excel_file = "LiDAR_Data_collection.xlsx"
        formatted_data = convert_excel_to_sensor_format(excel_file)
        
        print("Formatted sensor data:")
        print(formatted_data)
        
        # Save to a file
        with open('formatted_sensor_data.txt', 'w') as f:
            f.write(str(formatted_data))
            
        print(f"\nSuccessfully processed {len(formatted_data)} readings")
        print("Data has been saved to 'formatted_sensor_data.txt'")
            
    except FileNotFoundError:
        print(f"Error: Could not find the Excel file '{excel_file}'")
    except Exception as e:
        print(f"Error processing file: {str(e)}")

if __name__ == "__main__":
    main()
