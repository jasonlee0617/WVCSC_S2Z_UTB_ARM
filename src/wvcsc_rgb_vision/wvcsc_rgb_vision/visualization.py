import cv2


def draw_detection(image, bbox, target_xy, label, score):
    x, y, width, height = bbox
    cv2.rectangle(image, (x, y), (x + width, y + height), (0, 220, 255), 2)
    center = (int(target_xy[0]), int(target_xy[1]))
    cv2.circle(image, center, 5, (0, 0, 0), -1)
    cv2.circle(image, center, 3, (0, 255, 0), -1)
    cv2.putText(
        image, f'{label} {score:.2f}', (x, max(18, y - 6)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 2)
