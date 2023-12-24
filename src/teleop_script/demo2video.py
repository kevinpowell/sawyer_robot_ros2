'''
read .pkl file and save the imgs as video
'''

import pickle as pkl
import numpy as np 
import cv2
import argparse
import os 
import glob
import tqdm
import imageio 

def main(path, savedir):
    '''
    if path is a directory, list all the files inside
    '''
    paths=[path]
    if os.path.isdir(path):
        if path[-1]!='/':
            path=path+'/'
        paths=glob.glob(path+'*')

    print('paths: ', paths)

    for demo_name in paths:
        try:
            with open(demo_name, 'rb') as f:
                demo = pkl.load(f)
            imgs=demo['imgs']
        except Exception as e:
            print(f'Error: {e}')
            print(f'skip file: {demo_name}')
            continue

        print('demo_name: ', demo_name)
        fn=demo_name.split('/')[-1].strip()
        fn=fn.replace('.pkl', '.mp4')
        savepath = savedir+'/'+fn 
        # print('fn=', fn, savepath)

        writer = imageio.get_writer(savepath, fps=30)

        for img in imgs:
            image_wrist, image_front=img
            image=np.concatenate([image_wrist, image_front], axis=1) 
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) 
            writer.append_data(image_rgb)
        writer.close()


if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--src", type=str, default=0,  help="src file or directory", required=True)
    parser.add_argument("-dir", "--savedir", type=str, required=True)
    args=parser.parse_args()
    print('args=', args)
    main(args.src, args.savedir)

# python3 demo2video.py -f /home/carl/data_sawyer/block/11_09_2023_21_23_131.pkl --savedir  /home/carl/data_sawyer/block_videos
# python3 demo2video.py -f /home/carl/data_sawyer/block --savedir  /home/carl/data_sawyer/block_videos
# python3 demo2video.py -f /home/carl/data_sawyer/dclose --savedir  /home/carl/data_sawyer/videos_dclose
