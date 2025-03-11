#!/usr/bin/env python3
# coding=utf-8

'''
# Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2012-2023. All rights reserved.
# daplink_create_HiSpark_Trace_Firmware.py Function implementation: Used to generate the version upgrade package.
'''
import struct
import sys
import os
import stat
import pathlib

MAGIC_NUMBER = 0xAA55A55A
HIC_ID = 0
FILE_TYPE = 0x9B939E8F
VER = 1
FACTORY_ID = 0
CHIP_ID = 0
FLASH_TYPE = 0
FLASH_CRC = 0xFFFFFFFF


def packet_fill(dst, fill_len, val):
    while fill_len >= 32:
        dst.write(struct.pack('IIIIIIII', val, val, val, val, val, val, val, val))
        fill_len = fill_len - 32
    while fill_len >= 4:
        dst.write(struct.pack('I', val))
        fill_len = fill_len - 4
    while fill_len > 0:
        dst.write(struct.pack('B', val & 0xFF))
        fill_len = fill_len - 1


def add_loader_header(file_len):
    header = []
    header.append(MAGIC_NUMBER)
    header.append(HIC_ID)
    header.append(FILE_TYPE)
    header.append(VER)
    header.append(FACTORY_ID)
    header.append(CHIP_ID)
    header.append(FLASH_TYPE)
    header.append(file_len)
    header.append(FLASH_CRC)
    for _ in range(3):
        header.append(0)
    return header


def packet_bin(output_path, input_list):
    path_list = []
    burn_size_list = []
    image_size_list = []
    for item in input_list:
        path, burn_size = item.split("|")
        image_size = os.path.getsize(path)
        path_list.append(path)
        burn_size_list.append(int(burn_size))
        image_size_list.append(image_size)

    header = add_loader_header(image_size)

    flags = os.O_RDWR | os.O_CREAT
    modes = stat.S_IWUSR | stat.S_IRUSR
    with os.fdopen(os.open(output_path, flags, modes), 'wb+') as file:
        for i in header:
            file.write(struct.pack('I', i))
        with open(path, 'rb+') as src:
            data = src.read()
            file.write(data)
        packet_fill(file, int(burn_size) - image_size, 0xFFFFFFFF)
        file.flush()


def main(argv):
    '''
    Function description: Combine loader.bin with image.bin into allinone.bin.
    '''
    curpath = pathlib.Path().cwd()
    sourcepatch = curpath.joinpath("./images/allinone_upgrade_1M_CRC32.bin")
    output_path = curpath.joinpath("./images/HiSpark-Trace_Firmware_forDAPLINK.bin")

    input_list = ["{}|{}".format(sourcepatch, 32 * 1024)]
    packet_bin(output_path, input_list)


if __name__ == "__main__":
    sys.exit(main(sys.argv))