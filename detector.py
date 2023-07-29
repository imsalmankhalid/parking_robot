import cv2
import threading

class WhiteRectangleDetector:
    def __init__(self, camera_index=-1, threshold=50, min_rectangle_area=100):
        self.camera_index = camera_index
        self.threshold = threshold
        self.min_rectangle_area = min_rectangle_area
        self.cap = None
        self.stop_detection = False
        self.rectangle_detected = 0

    def find_biggest_contour(self, contours):
        max_area = -1
        biggest_contour = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                biggest_contour = contour
        return biggest_contour

    def detect_white_rectangles(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, self.threshold, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        white_rectangles = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_rectangle_area:
                white_rectangles.append(contour)

        return white_rectangles

    def draw_bounding_boxes(self, frame, rectangles):
        for rectangle in rectangles:
            x, y, w, h = cv2.boundingRect(rectangle)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    def is_rectangle_detected(self):
        """
        Returns True if at least one white rectangle was detected in the last processed frame,
        otherwise returns False.
        """
        # You can create a flag to keep track of whether a rectangle was detected or not in the last frame
        # For simplicity, let's assume the flag is called 'rectangle_detected'.
        # You should update this flag inside the 'start_detection()' method after rectangle detection.
        return self.rectangle_detected

    def start_detection(self):
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L)

        while not self.stop_detection:
            ret, frame = self.cap.read()
            if not ret:
                break
            frame = cv2.flip(frame, 0)
            # Convert frame to grayscale and flip it
            flipped_gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            flipped_gray_frame = cv2.flip(flipped_gray_frame, 1)

            # Find the biggest black region
            _, binary_frame = cv2.threshold(flipped_gray_frame, self.threshold, 255, cv2.THRESH_BINARY_INV)
            contours, _ = cv2.findContours(binary_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            biggest_contour = self.find_biggest_contour(contours)

            # Inside the black region, detect white rectangles
            white_rectangles = []
            if biggest_contour is not None:
                x, y, w, h = cv2.boundingRect(biggest_contour)
                roi = frame[y:y + h, x:x + w]
                white_rectangles = self.detect_white_rectangles(roi)

            # Draw bounding boxes of white rectangles on the original frame
            self.draw_bounding_boxes(frame, white_rectangles)

            # Set the 'rectangle_detected' flag to True if white rectangles were detected,
            # otherwise set it to False.
            self.rectangle_detected = len(white_rectangles)

            # Display the resulting video frame
            cv2.imshow("White Rectangles Detection", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        self.cap.release()
        cv2.destroyAllWindows()

    def run_detection(self):
        def start_detection():
            self.start_detection()

        # Run the detection in a separate thread
        detection_thread = threading.Thread(target=start_detection)
        detection_thread.start()

        
    def cleanup(self):
        """Release resources and perform cleanup."""
        self.stop_detection = True  # Set the flag to stop the detection loop

        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
