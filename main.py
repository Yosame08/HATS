'''
required: python3, pytorch, C++ Toolchain(cmake, make, g++), libtorch for C++
modify before running:
    main.py:12-14 file_train file_valid file_test
        trajectory files for training, validation and test
    Common/definitions.h:6 TRACEFILE
        change this macro to ../ + file_test in main.py
    Common/definitions.h:12 TIMEZONE
    	change this macro to the time zone of the region to which the trajectories corresponds
    Common/definitions.h:16 RECOVER_INTERVAL
        change this macro to the time interval you want to recover
    0.HMM_preprocess/CMakeLists.txt
    2.Make_Data_For_Train/CMakeLists.txt
    4.HMM_final/CMakeLists.txt
    Tool_Rating_New/CMakeLists.txt
    Tool_Shortest_Path/CMakeLists.txt
        change cmake_minimum_required version to your cmake version
parameters to be specified to run the script:
    --threads x
        Use x threads for cpp programs. This argument doesn't affect torch.
run example:
    python main.py --threads 8
'''
import sys
import torch
import os
import subprocess
import argparse

file_train = 'train_input.txt'
file_valid = 'valid_input.txt'
file_test = 'test_sampled.txt'
thread = 1

def stat_num(filename):
    num = 0
    with open(filename, "r") as f:
        for line in f:
            info = line.split(' ')
            if len(info) == 1:
                num += 1
    return num

def file_find(directory, filename):
    return filename in os.listdir(directory)


# Python执行shell指令并实时输出shell中的内容
def exec_cmd(command):
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    # 实时输出
    while True:
        output = process.stdout.readline()
        if output == '' and process.poll() is not None:
            break
        if output:
            print(str(output.strip(), 'utf-8'))
    rc = process.poll()
    return rc


def exec_cmd_in(directory, command):
    # save the working directory now
    current_dir = os.getcwd()
    os.chdir(directory)

    try:
        print(f"[Execute] {command}")
        rc = exec_cmd(command)
    except subprocess.CalledProcessError as e:
        print(f"命令执行失败，错误信息：{e}")
        # switch to the original working directory
        os.chdir(current_dir)
        # throw an error
        raise RuntimeError("命令执行失败") from e

    # switch to the original working directory
    os.chdir(current_dir)

def check_cpp(directory, filename):
    if not file_find(directory, filename):
        print(f"Can't find executable ./{directory}/{filename}, trying to make...")
        try:
            exec_cmd_in(directory, "cmake . && make")
        except:
            print(f"cmake . && make in {directory} failed.\n"
                  "If it was caused by cmake version, you can modify CMakeLists.txt manually.")
            exit(-1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Main script to run trajectory recover.\n"
                                                 "Example usage: python main.py --threads 8")
    parser.add_argument('--threads', type=int, default=1,
                        help="Number of threads for CPP code to use. This option don't effect python code(torch library)."  )
    args = parser.parse_args()

    # Step 0: Check if all cpp files are compiled and check system environment
    print("Check system environment and input files")
    check_cpp('0.HMM_preprocess', 'HMM_pre')
    check_cpp('2.Make_Data_For_Train', 'Make_Data')
    check_cpp('4.HMM_final', 'HMM_final')
    check_cpp('Tool_Rating_New', 'Tool_Rating_New')
    check_cpp('Tool_Shortest_Path', 'Tool_Shortest_Path')
    print("[Info] All cpp files are compiled.")
    if torch.cuda.is_available():
        device = 'cuda'
    else:
        device = 'cpu'
    print(f"[Info] Use device {device}")
    num_train = stat_num(file_train)
    num_valid = stat_num(file_valid)
    num_test = stat_num(file_test)


    # Step 1: Run HMM for preprocess
    print("Step 1/5: Running HMM for preprocessing at high precision trajectories")
    exec_cmd_in('0.HMM_preprocess', f'./HMM_pre -th {args.threads} -t')
    exec_cmd_in('0.HMM_preprocess', f'./HMM_pre -th {args.threads} -v')

    # Step 2: Parse trace data
    print("Step 2/5: Embedding road vectors based on preprocessing results")
    exec_cmd_in('1.Embed_Roads', f'{sys.executable} main.py')

    # Step 3: Generate data for training
    print("Step 3/5: Fit parameters and generate training data")
    exec_cmd_in('2.Make_Data_For_Train', f'./Make_Data -th {args.threads}')

    # Step 4: Train models
    print("Step 4/5: Train models")
    exec_cmd_in('3.Models', f'{sys.executable} VelocityPred.py')

    # Step 5: Recover traces by HMM and models
    print("Step 5/5: Recover trajectories by LSHMM and models")
    exec_cmd_in('4.HMM_final', f'./HMM_final -th {args.threads}')
    
    # Rate the recovery results
    print("Rating")
    exec_cmd_in('Tool_Shortest_Path', './Tool_Shortest_Path')
