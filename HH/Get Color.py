import cv2
pic = r"C:\Users\Hen Zhi Qian\Pictures\Camera Roll\WIN_20180312_09_37_59_Pro.jpg"
img = cv2.imread(pic)

# The [x, y] for each right-click event will be stored here
right_clicks = list()

# This function will be called whenever the mouse is right-clicked
def mouse_callback(event, x, y, flags, params):
    # Right-click event value is 2
    if event == 2:
        global right_clicks

        # Store the coordinates of the right-click event
        right_clicks.append([x, y])

        # This just verifies that the mouse data is being collected
        # You probably want to remove this later
        print(right_clicks)

# width = GetSystemMetrics(0)
# height = GetSystemMetrics(1)
scale_width = 640 / img.shape[1]
scale_height = 480 / img.shape[0]
scale = min(scale_width, scale_height)
window_width = int(img.shape[1] * scale)
window_height = int(img.shape[0] * scale)
cv2.namedWindow('image', cv2.WINDOW_NORMAL)
cv2.resizeWindow('image', window_width, window_height)

#set mouse callback function for window
cv2.setMouseCallback('image', mouse_callback)

cv2.imshow('image', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
