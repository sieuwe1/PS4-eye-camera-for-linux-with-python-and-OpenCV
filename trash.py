    #dispc= (denoised-denoised.min())*255
    #dispC= dispc.astype(np.uint8)                                   # Convert the type of the matrix from float32 to uint8, this way you can show the results with the function cv2.imshow()
    #dispC_inverted = cv2.bitwise_not(dispC)
    #mask = np.zeros(dispC.shape, np.uint8)
    #dispC_inverted = np.where(dispC_inverted==255, dispC_inverted, 0)

    #blurred_img = cv2.GaussianBlur(dispC, (11,11), 0)
    #contours, hierarchy = cv2.findContours(dispC_inverted, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #cv2.drawContours(mask, contours, -1, (255,255,255),10)
    #output = np.where(mask!=255, dispC, 80)