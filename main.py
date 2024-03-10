# required: pytorch, libtorch(for C++, please modify CMakeLists.txt in HMM_final dir)
import sys

import torch
import os
import subprocess
import argparse

file_train = 'train_input.txt'
file_valid = 'valid_input.txt'
file_test = 'test_input.txt'
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


# 函数2：Python执行shell指令并实时输出shell中的内容
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
    # 保存当前工作目录
    current_dir = os.getcwd()
    os.chdir(directory)

    try:
        rc = exec_cmd(command)
    except subprocess.CalledProcessError as e:
        print(f"命令执行失败，错误信息：{e}")
        # 切换回原来的工作目录
        os.chdir(current_dir)
        # 抛出异常
        raise RuntimeError("命令执行失败") from e

    # 切换回原来的工作目录
    os.chdir(current_dir)

def check_cpp(directory, filename):
    if not file_find(directory, filename):
        print(f"Can't find executable ./{directory}/{filename}, trying to make...")
        try:
            exec_cmd_in(directory, "cmake . && make")
        except:
            print("cmake . && make Failed. If it was caused by cmake version, you can modify CMakeLists.txt manually.")
            exit(-1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Main script to run trajectory recover.\n"
                                                 "Example usage: python3 main.py --threads 8")
    parser.add_argument('--threads', type=int, default=1,
                        help="Number of threads for CPP code to use. This option don't effect python code(torch library)."  )
    args = parser.parse_args()

    # Step 0: Check if all cpp files are compiled and check system environment
    print("Step 0/5: Check system environment and input files")
    check_cpp('1.HMM_preprocess', 'HMM_pre')
    check_cpp('2.Lights_process', 'Lights')
    check_cpp('3.Make_Training_Data', 'Make_Data')
    check_cpp('5.HMM_final', 'HMM_final')
    print(" - All cpp files are compiled.")
    if torch.cuda.is_available():
        device = 'cuda'
    else:
        device = 'cpu'
    print(f" - Use device {device}")
    num_train = stat_num(file_train)
    num_valid = stat_num(file_valid)
    num_test = stat_num(file_test)


    # Step 1: Run HMM for preprocess
    print("Step 1/5: Run HMM for preprocess")
    exec_cmd_in('1.HMM_preprocess', f'./HMM_pre -tr {num_train} -th {args.threads} -t')
    exec_cmd_in('1.HMM_preprocess', f'./HMM_pre -tr {num_valid} -th {args.threads} -v')

    # Step 2: Parse trace data
    print("Step 2/5: Parse trace data for training")
    exec_cmd_in('2.Lights_process', f'./Lights -th {args.threads} -t')
    exec_cmd_in('2.Lights_process', f'./Lights -th {args.threads} -v')

    # Step 3: Generate data for training
    print("Step 3/5: Generate data for training")
    exec_cmd_in('3.Make_Training_Data', f'./Make_Data -th {args.threads} -t')
    exec_cmd_in('3.Make_Training_Data', f'./Make_Data -th {args.threads} -v')

    # Step 4: Train models
    print("Step 4/5: Train models")
    exec_cmd_in('4.Models', f'{sys.executable} TrafficPred.py')
    exec_cmd_in('4.Models', f'{sys.executable} VelocityPred.py')

    # Step 5: Recover traces by HMM and models
    print("Step 5/5: Recover traces by HMM and models")
    exec_cmd_in('5.HMM_final', f'./HMM_final -tr {num_test} -th {args.threads}')