import cv2
import numpy as np
from time import sleep


# detect 할 범위 수정
# 히스토그램 찾는 범위 수정
# detect하는 부분 1개로 수정
# 곡률 게산 수정 필요


def warped(img, top_right, top_left, bottom_right, bottom_left):
    #extract image dimensions
    img_size = (img.shape[1], img.shape[0])
    #set source points
    src = np.float32([[top_right],[top_left],[bottom_right],[bottom_left]])
    #define width and height
    w, h = img.shape[1], img.shape[0]
    #set destination points
    dst = np.float32([[w,0],[0,0],[w,h],[0,h]])
    # get a perspective transform matrix
    M = cv2.getPerspectiveTransform(src, dst)
    # get inverse matrix
    Minv = cv2.getPerspectiveTransform(dst, src)
    # warp original image
    warped = cv2.warpPerspective(img, M, img_size, flags=cv2.INTER_LINEAR)

    #return warped image and inverse matrix
    return warped, Minv


def color_and_gradient_threshold(img, side):

    gray_img = 255-cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray_img, (7, 7), 5)
    #cv2.imshow('Gau_Blur'+side, blurred)

    # Sobel x
    sobelx = cv2.Sobel(blurred, cv2.CV_64F, 1, 0) # Take the derivative in x
    sobel_postive_filtered = np.zeros_like(sobelx)
    sobel_postive_filtered[(sobelx <= -70)] = 255
    abs_sobelx = sobel_postive_filtered

    #abs_sobelx = np.absolute(sobelx) # Absolute x derivative to accentuate lines away from horizontal
    scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
    
    # Apply Sobel filter- x
    thresh_min = 90
    thresh_max = 255
    sobel_filtered = np.zeros_like(scaled_sobel)
    sobel_filtered[(scaled_sobel >= thresh_min) & (scaled_sobel <= thresh_max)] = 255
    cv2.imshow('Sobel'+side, sobel_filtered)

    # Apply Gaussian filter
    s_thresh_min = 30
    s_thresh_max = 255
    s_binary = np.zeros_like(blurred)
    s_binary[(blurred >= s_thresh_min) & (blurred <= s_thresh_max)] = 255
    gaussian = 255-cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 5,2)

    cv2.imshow('Gaussian'+side, gaussian)

    # Combine the two binary thresholds
    combined_binary = np.zeros_like(sobel_filtered)
    #combined_binary[(s_binary == 255) | (sxbinary == 255)] = 1
    combined_binary[(gaussian == 255) & (sobel_filtered == 255)] = 255

    return combined_binary

def binary(img, region, side):

    top_left, top_right, bottom_left,  bottom_right = region

    #Aplying Thresholds
    color_and_gradient = color_and_gradient_threshold(img, side)

    #Perspective Transform
    binary_warped, Minv = warped(color_and_gradient, top_right, top_left, bottom_right, bottom_left)
    color_warped, _ = warped(img, top_right, top_left, bottom_right, bottom_left)

    return binary_warped, color_warped, Minv

def fitlines(binary_warped, color_warped):
    
    # Choose the number of sliding windows
    nwindows = 9
    # Set height of windows
    window_height = int(binary_warped.shape[0]/nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Current positions to be updated for each window
    # Set the width of the windows +/- margin
    margin = 15
    # Set minimum number of pixels found to recenter window
    minpix = 20
    # Create empty lists to receive lane pixel indices
    lane_inds = []

    points_info = []
    temp_xpos = []
    temp_ypos = []
    x_prev = -1
    # 위에서부터 아래로
    for window in range(nwindows):
        
        # Identify window boundaries in x and y (and right and left)
        win_y_low = binary_warped.shape[0] - (window+1)*window_height
        win_y_high = binary_warped.shape[0] - window*window_height
        #temp1 = np.sum([binary_warped[win_y_low+i][:] for i in range(window_height)], axis=0).nonzero()[0]
        temp2 = np.sum([binary_warped[win_y_low+i][:] for i in range(int(window_height/2))], axis=0)
        temp3 = np.sum([binary_warped[win_y_low+i][:] for i in range(int(window_height/2), window_height)], axis=0)
        val1, val2 = np.argmax(temp2), np.argmax(temp3)
        x_current_ = int((val1 + val2)/2)

        if (val1 != 0) and (val2 != 0) and (not(x_prev != -1 and abs(x_current_-x_prev)>80)):
            
            # if np.abs(np.argmax(temp3)-np.argmax(temp2)) > 100:
            #     print('doube', np.argmax(temp2), np.argmax(temp3))
            #x_current = int((temp[0]+temp[-1])/2)
            x_current = x_current_
            temp_xpos.append(x_current)
            temp_ypos.append(int((win_y_high+win_y_low)/2))
            win_x_low = x_current - margin
            win_x_high = x_current + margin

            points_info.append((x_current, int((win_y_high+win_y_low)/2)))

            # Draw the windows on the visualization image
            cv2.rectangle(color_warped,(win_x_low, win_y_low),(win_x_high, win_y_high),(0,255,0), 2)

            # Identify the nonzero pixels in x and y within the window
            good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_y_high)).nonzero()[0]

            #print('window',window,':',good_inds)
            
            # Append these indices to the lists
            lane_inds.append(good_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            # if len(lane_inds) > minpix:
            #     x_current = int(np.mean(nonzerox[lane_inds]))

            x_prev = x_current
    # Concatenate the arrays of indices
    try:
        lane_inds = np.concatenate(lane_inds)

    except:
        pass

    # Extract line pixel positions
    x_pix_pos = nonzerox[lane_inds]
    y_pix_pos = nonzeroy[lane_inds]

    # Fit a second order polynomial to each
    if len(x_pix_pos) == 0:
        left_fit = [False]
    else:
        #left_fit = np.polyfit(y_pix_pos, x_pix_pos, 2)
        left_fit = np.polyfit(temp_ypos, temp_xpos, 2)

    #out_img[nonzeroy[lane_inds], nonzerox[lane_inds]] = [255, 255, 0]

    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )

    return left_fit, color_warped, np.array(temp_ypos), np.array(temp_xpos), ploty, points_info

def draw_tan_line(img, p1, p2):
    height = img.shape[0]
    x_3, y_3 = 0, 0
    if p2[0]-p1[0] != 0:
        slope = (p2[1]-p1[1])/(p2[0]-p1[0])
        y_intercept = p1[1]-slope*p1[0]
        x_intercept = (-y_intercept)/slope
        ##### Eqn: y = slope*x + y_intercept #####
        if x_intercept < 0:
            x_3 = 0
            y_3 = y_intercept
        elif x_intercept > img.shape[1]:
            x_3 = img.shape[1]-1
            y_3 = int(slope*x_3 + y_intercept)
        else:
            x_3 = x_intercept
            y_3 = 0
        angle = np.arctan(slope) * 180 / np.pi
        angle = angle if angle >= 0 else 180 + angle
        p3 = (int(x_3), int(y_3))
    else:
        p1 = (p1[0], p1[1])
        p3 = (p1[0], 0)
        angle = 90

    cv2.line(img,p1,p3,(0,0,255),1, cv2.LINE_AA)
    return img, angle

def curvatures(y_pix_pos, x_pix_pos, ploty):

    # Define conversions in x and y from pixels space to meters
    ym_per_pix = 30/480 # meters per pixel in y dimension
    xm_per_pix = 40/640 # meters per pixel in x dimension

    y_eval = np.max(ploty)

    # Fit new polynomials to x,y in world space
    #fit_cr = np.polyfit(y_pix_pos*ym_per_pix, x_pix_pos*xm_per_pix, 2)
    ## Calculate the new radii of curvature
    #curverad = ((1 + (2*fit_cr[0]*y_eval*ym_per_pix + fit_cr[1])**2)**1.5) / np.absolute(2*fit_cr[0])
    
    if len(x_pix_pos) <= 1:
        return 0
    else:
        fit_cr = np.polyfit(y_pix_pos, x_pix_pos, 1)
        #print(len(y_pix_pos), '|', len(x_pix_pos))

        angle = np.arctan(fit_cr[0]) * 180 / np.pi
        curverad = 0.15*angle + 90 + 3
        print(fit_cr[1])

        # if fit_cr[1] is big, it means it's on the right lane
        if fit_cr[1]>140:
            curverad -= 13

        elif fit_cr[1]<60:
            curverad += 8

        elif fit_cr[1]<30:
            curverad += 20
        #curverad

        return curverad

def back_warp(img_shape, warped, left_fit, ploty, curverad, Minv):

    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]

    # Recast the x and y points into usable format for cv2.fillPoly()
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    #pts = np.hstack((pts_left, pts_right))

    # Draw the lane onto the warped blank image
    #cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0))

    # Warp the blank back to original image space using inverse perspective matrix (Minv)
    result = cv2.warpPerspective(warped, Minv, (img_shape[1], img_shape[0])) 

    # Creating Text and set font parameters
    TextL = "Curvature: " + str(int(curverad)) + " cm"
    fontScale=0.4
    thickness=1
    fontFace = cv2.FONT_HERSHEY_COMPLEX

    # Using CV2 putText to write text into images
    cv2.putText(result, TextL, (5,10), fontFace, fontScale,(0,255,255), thickness,  lineType = cv2.LINE_AA)
    #cv2.putText(result, 'Hanwool Lee', (0,60), fontFace, 0.7,(0,255,255), thickness,  lineType = cv2.LINE_AA)

    return result

def pipeline(img, region, prev_curvature, prev_fit, side):

    # creating a Binary Undistorced Warped Image
    binary_warped, color_warped, Minv = binary(img, region, side)

    # Fiting Lines
    left_fit, out_img, y_pix_pos, x_pix_pos, ploty , points_info = fitlines(binary_warped, color_warped)
    
    # if it didn't detect any lines, use previous loop value
    if left_fit[0] == False:
        left_fit = prev_fit

    # Draw tangential line on the image

    try:
        out_img, angle = draw_tan_line(out_img, points_info[0], points_info[1])
    except:
        angle=90

    # Calulating the left and right lines curvatures
    try:
        curverad = curvatures(y_pix_pos, x_pix_pos, ploty)

    except:
        curverad = prev_curvature

    # Draw Lane between road lines
    result_lane = back_warp((img.shape[0], img.shape[1]), out_img, left_fit, ploty, curverad, Minv)
    return result_lane, curverad, left_fit, angle

############################ main ##################################
def main():
    # [우, 하]
    top_left = [200, 200]
    top_right = [420, 200]
    bottom_right = [580, 340]
    bottom_left = [30, 380]
    top_left = [0, 0]
    top_right = [640, 0]
    bottom_right = [640, 480]
    bottom_left = [0, 480]
    top_left = [0, 0]
    top_right = [240, 0]
    bottom_right = [240, 320]
    bottom_left = [0, 320]
    region = (top_left, top_right, bottom_left, bottom_right)


    #capture1 = cv2.VideoCapture(r'http://192.168.137.200:81/stream')
    #capture2 = cv2.VideoCapture(r'http://192.168.137.202:81/stream')

    capture1 = cv2.VideoCapture(2)
    capture1.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    capture1.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    capture2 = cv2.VideoCapture(0)
    capture2.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    capture2.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    #capture = cv2.VideoCapture(r'C:\Users\Hanwool\OneDrive - 고려대학교\Capstone\Lane_detection\sample\project_video.mp4')

    curvature_L, curvature_R = 0,0
    left_fit_L = [2.60030797e-05, -1.62531407e-01,  4.67357700e+02]
    left_fit_R = [2.60030797e-05, -1.62531407e-01,  4.67357700e+02]
    while True:

        _, frame1 = capture1.read()
        _, frame2 = capture2.read()

        #img = cv2.resize(frame, dsize=(640, 480), interpolation=cv2.INTER_AREA)
        frame_L = frame1
        frame_R = frame2
        frame_L = cv2.rotate(frame_L, cv2.ROTATE_90_COUNTERCLOCKWISE)
        frame_R = cv2.rotate(frame_R, cv2.ROTATE_90_CLOCKWISE)

        #rc = np.array([top_left, bottom_left, bottom_right, top_right], np.int32)
        
        img_L, curvature_L, left_fit_L, angle_L = pipeline(frame_L, region, curvature_L, left_fit_L, 'L')
        img_R, curvature_R, left_fit_R, angle_R = pipeline(frame_R, region, curvature_R, left_fit_R, 'R')

        #img3 = cv2.hconcat([im, img2])
        #img3 = cv2.resize(img3, dsize=(960, 720))

        #print(str((curvature1+curvature2)/2) + 'cm')

        #img = cv2.polylines(img, [pts], True, (0, 255, 0), 2)

        #cv2.imshow('recog1', img3)
        img_R = cv2.resize(img_R, (480, 640))
        img_L = cv2.resize(img_L, (480, 640))
        cv2.imshow('R', img_R)
        cv2.imshow('L', img_L)
        cv2.waitKey(1)

    #img = cv2.imread(r"./1lane.png", cv2.IMREAD_UNCHANGED)


if __name__ == "__main__":
    main()
