import os
import glob
import cv2
import numpy as np

Your_color = "Yellow"

def detect_single_color(imageFrame, color_name, lower_range, upper_range, color_display):
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    color_mask = cv2.inRange(hsvFrame, lower_range, upper_range)

    kernel = np.ones((5, 5), "uint8")
    color_mask = cv2.dilate(color_mask, kernel)

    contours, _ = cv2.findContours(color_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 800:
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), color_display, 2)
            cv2.putText(imageFrame, f"{color_name} Colour", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color_display)
    return imageFrame, color_mask

def compute_color_percentage(mask):
    colored_pixels = np.count_nonzero(mask)
    total_pixels = mask.size
    return (colored_pixels / total_pixels) * 100.0

def main():
    photos_dir = os.path.join(os.path.dirname(__file__), "RaceCoursePhotos")
    image_paths = sorted(glob.glob(os.path.join(photos_dir, "*.*")))

    if not image_paths:
        print(f"No images found in: {photos_dir}")
        return

    # HSV range for yellow; adjust if needed
    # H: 20–40 (approx), S: 100–255, V: 100–255
    my_color_lower = np.array([3, 41, 117], dtype=np.uint8)
    my_color_upper = np.array([41, 204, 255], dtype=np.uint8)

    # Output folder for overlays (optional)
    output_dir = os.path.join(photos_dir, "outputs")
    os.makedirs(output_dir, exist_ok=True)

    for img_path in image_paths:
        img = cv2.imread(img_path)
        if img is None:
            print(f"Skipping unreadable file: {img_path}")
            continue

        result_frame, mask = detect_single_color(img.copy(), Your_color, my_color_lower, my_color_upper, (0, 255, 0))
        percent_yellow = compute_color_percentage(mask)
        print(f"{os.path.basename(img_path)}: {percent_yellow:.2f}% yellow")

        # Optional: save visualization and mask
        base = os.path.splitext(os.path.basename(img_path))[0]
        cv2.imwrite(os.path.join(output_dir, f"{base}_overlay.jpg"), result_frame)
        cv2.imwrite(os.path.join(output_dir, f"{base}_mask.png"), mask)

    #check image for percentage of yellow color
    #THIS IS WHAT COMMANDS THE ARDUINO
    if percent_yellow >10:
        #Continue in direction
        return True
    else:
        #stop and correct direction
        return False
    

if __name__ == "__main__":
    main()