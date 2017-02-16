#!/usr/bin/env python
import os 
import sys
import subprocess
from multiprocessing import Pool

root_input_dir = os.path.abspath(sys.argv[1])
root_output_dir = sys.argv[2]
sample_type = sys.argv[3]
num_samples = sys.argv[4]
width = sys.argv[5]
height = sys.argv[6]

if (not os.path.isabs(root_output_dir)):
    root_output_dir = os.getcwd() + '/' + root_output_dir

bin_path = os.getcwd() + '/build/bin/main'

root_output_dir = os.path.abspath(root_output_dir)

print('Root input dir: %s' % root_input_dir)
print('Root output dir: %s' % root_output_dir)

def render_views(arg):
    input_path, output_path, image_width, image_height = arg
    subprocess.call([bin_path, input_path, output_path, sample_type,\
        num_samples, str(image_width),str(image_height)])

if __name__ == '__main__':
    pool = Pool(processes=4)  
    worklist = []
    for dir_name, dir_list, file_list in os.walk(root_input_dir):
        dir_name=os.path.abspath(dir_name)
        print('Found dir: %s' % dir_name)
        for file_name in file_list:
            name, extension = os.path.splitext(file_name)
            if extension == '.off':
                print('\tname=%s ext=%s' % (name, extension))
                input_path = dir_name + '/' + file_name
                output_path=dir_name[len(root_input_dir)+1:]
                full_output_path = root_output_dir + '/' + output_path
                if not os.path.exists(full_output_path):
                    os.makedirs(full_output_path)
                print('\toutput_path=%s/%s' % (root_output_dir, output_path))
                worklist.append((input_path, full_output_path, width, height))
    pool.map(render_views, worklist)