'''
read .pkl file view the images
'''

import pickle as pkl
import numpy as np 
import cv2
import argparse
import time

def main(path):
   
    with open(path, 'rb') as f:
        demo = pkl.load(f)
    imgs=demo['imgs']

    for img in imgs:
        image_wrist, image_front=img
        image=np.concatenate([image_wrist, image_front], axis=1)
        cv2.imshow('frame',image)
        if cv2.waitKey(1) == 27: 
            print('UI closed')
            break  # esc to quit
        time.sleep(1/30)

    cv2.destroyAllWindows() 


if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--file", type=str, default=0,  help="src file or directory", required=True)
    args=parser.parse_args()
    print('args=', args)
    main(args.file)

#  python3 view_demo.py -f /media/ns/DatasetDrive/data_sawyer/spoon/09_14_2023_22_33_0.04070901870727539_324.pkl

