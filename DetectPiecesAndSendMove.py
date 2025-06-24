# Darion
# Capstone Project
#
# Raspberry Pi Pipline:
# 1. intialize stockfish
# 2. intialize serial communication
# 3. detect board and warp in either auto or manual mode
# 3a. skip auto and manual mode and manually input fen
# 4. fen sends to stockfish for best move
# 5. best move sent via serial to mega
# 6. mega carries out best move and sends gripper serial logic when appropriate
# 7. repeat 

import time
import serial
import cv2 as cv
import numpy as np
from stockfish import Stockfish

# initalize stockfish
stockfish = Stockfish(path="/usr/games/stockfish") 

# connection the arduino mega
mega_serial = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)

# connection to arduino uno gripper
gripper_serial = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
time.sleep(2)

# starting mode and flags
mode = "auto"
manual_fen_override = None
clicked_points = []

# width of warp and height, rectangle
warp_width = 400
warp_height = 300

# open video (0 default for pi)
cap = cv.VideoCapture(0)
cv.namedWindow("Original")

#  piece colors HSV
piece_colors = {
    "k": ([90, 193, 74], [130, 255, 154]),
    "q": ([0, 93, 157], [35, 173, 237]),
    "r": ([18, 116, 192], [58, 196, 255]),
    "b": ([48, 130, 129], [88, 210, 209]),
    "n": ([13, 68, 176], [53, 148, 255]),
    "p": ([151, 19, 211], [179, 99, 255]),
    "K": ([143, 83, 215], [179, 163, 255]),
    "Q": ([0, 130, 210], [23, 210, 255]),
    "R": ([2, 20, 133], [42, 67, 213]),
    "B": ([61, 191, 128], [101, 255, 208]),
    "N": ([130, 102, 102], [170, 182, 182]),
    "P": ([77, 72, 134], [117, 152, 214])
}

def wait_for_mega(signal, timeout=1000):
    start_time = time.time()
    while time.time() - start_time < timeout:
        if mega_serial.in_waiting:
            response = mega_serial.readline().decode().strip()
            print(f"Arduino: {response}")
            if response == signal:
                return True
    print(f"Timeout waiting for: {signal}")
    return False

# sends OPEN to uno through serial
def grip_open():
    gripper_serial.write(b'OPEN\n')
    gripper_serial.flush()

# sends CLOSE to uno through serial
def grip_close():
    gripper_serial.write(b'CLOSE\n')
    gripper_serial.flush()


def send_to_robot(move):
    # debug
    print(f"Sending move to robot: {move}")
    mega_serial.write((move + '\n').encode())
    mega_serial.flush()

    # close gripper is ready
    if wait_for_mega("READY_TO_GRIP"):
        grip_close()
        time.sleep(0.7)

    # release gripper if ready
    if wait_for_mega("READY_TO_RELEASE"):
        grip_open()
        time.sleep(0.7)

# detect pieces from img
def detect_piece(square_img):
    hsv = cv.cvtColor(square_img, cv.COLOR_BGR2HSV)
    # for each piece and lower, upper bound in piece_colours loop
    for piece, (lower, upper) in piece_colors.items():
        # apply mask
        mask = cv.inRange(hsv, np.array(lower), np.array(upper))
        # set kernel and reduce noise
        # https://docs.opencv.org/4.x/d9/d61/tutorial_py_morphological_ops.html
        kernel = np.ones((5, 5), np.uint8)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
        # check for at least 20 pixels
        if cv.countNonZero(mask) > 20:
            return piece
    return ""

def validate_fen(fen):
    ranks = fen.split(" ")[0].split("/")
    if len(ranks) != 8:
        return False
    for rank in ranks:
        squares = 0
        for char in rank:
            if char.isdigit():
                # A-H
                squares += int(char)
            elif char.isalpha():
                #1-8
                squares += 1
        if squares != 8:
            return False
    return True

def draw_board_labels(warped):
    fen_rows = []
    square_width = warped.shape[1] // 8
    square_height = warped.shape[0] // 8

    for row in range(8):
        fen_row = ""
        empty_counter = 0
        # for 8 colums loop
        for col in range(8):
            x_start = col * square_width
            y_start = row * square_height
            square = warped[y_start:y_start + square_height, x_start:x_start + square_width]
            # check for detected piece
            piece = detect_piece(square)
            # no piece, increment empty_counter
            if piece == "":
                empty_counter += 1
            else:
                # if there is a piece present, putText to display
                if empty_counter > 0:
                    fen_row += str(empty_counter)
                    empty_counter = 0
                fen_row += piece
                cv.putText(warped, piece, (x_start + 10, y_start + 30),
                            cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv.rectangle(warped, (int(x_start), int(y_start)),
                          (int(x_start + square_width), int(y_start + square_height)),
                          (0, 255, 0), 1)
        if empty_counter > 0:
            fen_row += str(empty_counter)
        fen_rows.append(fen_row)
    # add end of fen string
    fen = "/".join(fen_rows) + " w KQkq - 0 1"
    return fen, warped

# append mouseclicks until 4 corners have been clicked
def mouse_callback(event, x, y, flags, param):
    global clicked_points
    if event == cv.EVENT_LBUTTONDOWN and mode == "manual_clicks" and len(clicked_points) < 4:
        clicked_points.append([x, y])
        print(f"Clicked corner {len(clicked_points)}: ({x}, {y})")

cv.setMouseCallback("Original", mouse_callback)
fen = None


while True:
    ret, frame = cap.read()
    if not ret:
        break
    # resize frame, easier for pi
    frame = cv.resize(frame, (640, 480))
    display_frame = frame.copy()
    # gray, blur and find edges and contours
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    blur = cv.GaussianBlur(gray, (5, 5), 0)
    edges = cv.Canny(blur, 80, 200)
    contours, _ = cv.findContours(edges, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    # init largest
    largest_contour = None
    max_area = 0
    # if auto, look for largest contour
    if mode == "auto":
        for cnt in contours:
            #https://docs.opencv.org/4.x/d3/dc0/group__imgproc__shape.html#ga0012a5fdaea70b8a9970165d98722b4c
            approx = cv.approxPolyDP(cnt, 0.02 * cv.arcLength(cnt, True), True)
            area = cv.contourArea(cnt)
            # if all four contours and bigger then reset max area and largest contour
            if len(approx) == 4 and area > max_area:
                x, y, w, h = cv.boundingRect(approx)
                aspect_ratio = w / float(h)
                if 0.8 < aspect_ratio < 1.4:
                    max_area = area
                    largest_contour = approx

    pts = None
    if largest_contour is not None and mode == "auto":
        pts = largest_contour.reshape(4, 2)
    # set to clicked points if clicked points have been set
    elif mode == "manual_clicks" and len(clicked_points) == 4:
        pts = np.array(clicked_points, dtype="float32")

    if pts is not None:
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]
        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]

        dst = np.array([
            [0, 0],
            [warp_width - 1, 0],
            [warp_width - 1, warp_height - 1],
            [0, warp_height - 1]
        ], dtype="float32")
        # warp perspective
        M = cv.getPerspectiveTransform(rect, dst)
        warped = cv.warpPerspective(frame, M, (warp_width, warp_height))

        fen, warped = draw_board_labels(warped)

        # show warped board
        cv.imshow("Warped Board", warped)

        # show FEN on original feed
        if fen:
            cv.putText(display_frame, f"FEN: {fen[:60]}", (10, 60), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

    # display mode
    cv.putText(display_frame, f"MODE: {mode.upper()}", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    cv.imshow("Original", display_frame)

    key = cv.waitKey(1) & 0xFF

    # if q quit
    if key == ord('q'):
        break
    # if s then send current fen to stockfish (prone to crashing!!)
    elif key == ord('s') and fen:
        if validate_fen(fen):
            print(f"Sending FEN to Stockfish: {fen}")
            stockfish.set_fen_position(fen)

            # special case for opening fools mate
            if fen.startswith("rnbqkbnr/pppppppp/8/8/6P1/8/PPPPPP1P/RNBQKBNR b KQkq - 0 1"):
                best_move = "e7e5"
                print("Detected special FEN - forcing move to:", best_move)
            # get best move
            else:
                best_move = stockfish.get_best_move()
            # debug
            print("Stockfish best move:", best_move)
            # send to robot to do
            send_to_robot(best_move)
        else:
            # debug
            print("Invalid FEN.")
    # if f then manually enter FEN
    elif key == ord('f'):
        mode = "manual_fen"
        manual_fen_override = input("Enter custom FEN string here: ")
        print(f"Manual FEN loaded: {manual_fen_override}")
        # set fen position and manually overide for fools mate
        if validate_fen(manual_fen_override):
            stockfish.set_fen_position(manual_fen_override)
            # fools mate for demo
            if manual_fen_override == "rnbqkbnr/pppppppp/8/8/6P1/8/PPPPPP1P/RNBQKBNR b KQkq - 0 1":
                best_move = "e7e5"
                print("Detected special FEN - forcing move to:", best_move)
            # if past opening of fools mate 
            else:
                best_move = stockfish.get_best_move()
            print("Stockfish best move:", best_move)
            # send to robot to do
            send_to_robot(best_move)
        else:
            print("Invalid FEN entered.")
        # return to auto mode
        mode = "auto"
        print("Returning to AUTO mode")
    # switch to manual mode and reset clicked points
    elif key == ord('m'):
        mode = "manual_clicks"
        clicked_points = []
        print("Switched to MANUAL CLICK MODE")
    #switch to auto mode
    elif key == ord('a'):
        mode = "auto"
        clicked_points = []
        print("Switched to AUTO MODE")
    # reset clicked points when in manual mode
    elif key == ord('r') and mode == "manual_clicks":
        clicked_points = []
        print("Manual clicks reset.")

cap.release()
cv.destroyAllWindows()


