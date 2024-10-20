import cv2
import math
import serial.tools.list_ports
import serial

import torch  # Import torch for GPU availability check
from ultralytics import YOLO
import cvzone
import time

def get_user_port():
    """Lists available COM ports and prompts user for selection."""
    ports = serial.tools.list_ports.comports()
    ports_list = []

    for one in ports:
        ports_list.append(str(one))
        print(str(one))

    com = input("Select Com Port for Arduino (#): ")
    return str(com)

def main():
    serial_inst = None  
    
    try:
        # Check if a GPU is available and use it if possible
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print(f"Using device: {device}")

        port = get_user_port()
        serial_inst = serial.Serial(port, 115200, timeout=1)  # Open serial port

        print(f"Opened serial port: {port}")
        time.sleep(1)

        model = YOLO("ppe.pt")
        model.to(device) 
        cap = cv2.VideoCapture(0)  
        cap2 = cv2.VideoCapture(1) 
        closest_ball = None
        current_mode = None
        previous_motor_command = None 
        previous_mode = None  

        classNames = ["B", "P", "R", "S"]
        frame_count = 0
        skip_frames = 0 
        myColor = (0, 0, 255)

        while True:
            if serial_inst.inWaiting() > 0:
                datapacket = serial_inst.readline().decode('utf-8').strip()
                current_mode = datapacket

                # Only print if the mode has changed
                if current_mode != previous_mode:
                    print(f"Received current mode from Arduino: '{current_mode}'")
                    previous_mode = current_mode

            if current_mode in ["start", "L", "F", "R", "S"]:
                success, img = cap.read()
                if not success:
                    break

                results = model(img, stream=True)
                red_balls_in_silos = {}

                for r in results:
                    boxes = r.boxes
                    for box in boxes:
                        x1, y1, x2, y2 = box.xyxy[0]
                        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                        conf = math.ceil((box.conf[0] * 100)) / 100
                        cls = int(box.cls[0])
                        current_class = classNames[cls]

                        if conf > 0.5 and current_class == 'S':
                            silo_box = (x1, y1, x2, y2)
                            if silo_box not in red_balls_in_silos:
                                red_balls_in_silos[silo_box] = 0
                            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 3)

                        elif conf > 0.6 and current_class == 'R':
                            already_counted = False
                            for silo_box, count in red_balls_in_silos.items():
                                silo_x1, silo_y1, silo_x2, silo_y2 = silo_box
                                if x1 >= silo_x1 and y1 >= silo_y1 and x2 <= silo_x2 and y2 <= silo_y2:
                                    already_counted = True
                                    break
                            if not already_counted:
                                for silo_box in red_balls_in_silos.keys():
                                    silo_x1, silo_y1, silo_x2, silo_y2 = silo_box
                                    if x1 >= silo_x1 and y1 >= silo_y1 and x2 <= silo_x2 and y2 <= silo_y2:
                                        red_balls_in_silos[silo_box] += 1
                                        
                                        break
                            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 3)

                if red_balls_in_silos:
                    max_balls_silo = max(red_balls_in_silos, key=red_balls_in_silos.get)
                    
                    x1, y1, x2, y2 = max_balls_silo
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2

                    # Divide the frame into 5 sections along the y-axis
                    frame_height = img.shape[0]
                    section_height = frame_height // 5

                    turn_threshold = img.shape[1] // 3

                    if center_x < img.shape[1] // 2 - turn_threshold:
                        motor_command = 'L'
                    elif center_x > img.shape[1] // 2 + turn_threshold:
                        motor_command = 'R'
                    else:
                        if y1 < section_height:
                            motor_command = 'S'
                        else:
                            motor_command = 'F'

                    if motor_command != previous_motor_command:
                        serial_inst.write(motor_command.encode())
                        previous_motor_command = motor_command
                        print(f"Sending motor command: {motor_command}")
                        print(x1, y1, x2, y2)
                        print("Center coordinates:", center_x, img.shape[1] // 2)

                cv2.imshow("Object Detection", img)
                cv2.waitKey(1)

            elif current_mode in ["g2g","g"]:
                motor_command = 'g'
                
                if motor_command != previous_motor_command:
                    serial_inst.write(motor_command.encode())
                    previous_motor_command = motor_command
                    print(f"Sending motor command: {motor_command}")

            elif current_mode in ["stop", "b", "l", "f", "r", "s"]:
                success, img = cap2.read()
                if not success:
                    break

                results = model(img, stream=True)
                red_balls = []

                for r in results:
                    boxes = r.boxes
                    for box in boxes:
                        x1, y1, x2, y2 = box.xyxy[0]
                        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                        conf = math.ceil((box.conf[0] * 100)) / 100
                        cls = int(box.cls[0])
                        currentClass = classNames[cls]

                        if conf > 0.5 and currentClass == 'R':
                            center_x = int(x1 + (x2 - x1) // 2)
                            center_y = int(y1 + (y2 - y1) // 2)
                            red_balls.append((center_x, center_y, conf))

                if red_balls:
                    closest_ball = min(red_balls, key=lambda ball: cv2.norm(ball[:2], (img.shape[1] // 2, img.shape[0] // 2)))
                    center_x, center_y, conf = closest_ball
                    distance_to_center = cv2.norm((center_x, center_y), (img.shape[1] // 2, img.shape[0] // 2))
                    turn_threshold = img.shape[1] // 5
                    distance_threshold = img.shape[0] // 10
                    frame_height = img.shape[0]
                    num_sections = 4  # Dividing the frame into 3 parts
                    section_height = frame_height // num_sections

                    if center_x < turn_threshold:
                        motor_command = 'r'
                    elif center_x > img.shape[1] - turn_threshold:
                        motor_command = 'l'
                    elif center_y > frame_height - section_height:
                        motor_command = 's'  # Stop if center_y is in the bottom section
                    else:
                        motor_command = 'f'  # Move forward if center_y is not in the bottom section

                else:
                    motor_command = 'b'

                if motor_command != previous_motor_command:
                    serial_inst.write(motor_command.encode())
                    previous_motor_command = motor_command
                    print(f"Sending motor command: {motor_command}")
                        
                for center_x, center_y, conf in red_balls:
                    cvzone.putTextRect(img, f'R {conf}', (center_x - 10, center_y - 10), scale=1, thickness=1,
                                       colorB=myColor,
                                       colorT=(255, 255, 255), colorR=myColor, offset=5)

                cv2.imshow("Image", img)
                cv2.waitKey(1)

    except serial.SerialException as e:
        print(f"Serial communication error: {e}")
        

    except Exception as e:
        print(f"Error: {e}")

    finally:
        if serial_inst is not None and serial_inst.is_open:
            serial_inst.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()
