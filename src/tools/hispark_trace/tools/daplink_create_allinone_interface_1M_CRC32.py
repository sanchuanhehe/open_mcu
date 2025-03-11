#!/usr/bin/env python3
# coding=utf-8

'''
# Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2012-2023. All rights reserved.
# daplink_create_allinone_interface_1M_CRC32.py: Used to merge A-core and M-core binary files. 
'''
import struct
import sys
import os
import stat
import pathlib
import zlib


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


def packet_bin_write_without_crc(dst, src, pad_len, pad_val):
    data = src.read()
    dst.write(data)
    packet_fill(dst, pad_len, pad_val)


def packet_bin_write_with_crc(dst, src, image_len, pad_len, pad_val):
    #构造magic
    magic_str = "HiSparkFW"
    magic_len = len(magic_str)
    magic_bytes = bytes(magic_str.encode('utf-8'))
    image_len += magic_len
    pad_len -= magic_len

    #写bin+magicbytes
    data = src.read()
    data += magic_bytes
    dst.write(data)

    #写bin对应CRC32
    crc = zlib.crc32(data) & 0xFFFFFFFF
    dst.write(struct.pack('I', crc))

    #剩余部分填充val，val可以为0xFF或者0x0
    packet_fill(dst, pad_len, pad_val)

    #写报文长度, 包含4个字节CRC
    image_len += 4
    dst.write(struct.pack('I', image_len))


def packet_bin(output_path, input_list, max_time):
    path_list = []
    burn_size_list = []
    image_size_list = []
    for item in input_list:
        path, burn_size = item.split("|")
        image_size = os.path.getsize(path)
        path_list.append(path)
        burn_size_list.append(int(burn_size))
        image_size_list.append(image_size)

    flags = os.O_RDWR | os.O_CREAT
    modes = stat.S_IWUSR | stat.S_IRUSR
    with os.fdopen(os.open(output_path, flags, modes), 'wb+') as file:
        times = 0
        start = burn_size_list[0] + burn_size_list[1] + 8 # boot size + 8字节文件长度
        for path in path_list:
            with open(path, 'rb+') as subfile:
                pad_len = burn_size_list[times] - image_size_list[times]
                pad_val = 0
                pad_len -= 8
                packet_bin_write_with_crc(file, subfile, image_size_list[times], pad_len, pad_val)

                times += 1
            if times >= max_time:
                break
        file.flush()


def main(argv):
    '''
    Function description: Combine loader.bin with image.bin into allinone.bin.
    '''
    curpath = pathlib.Path().cwd()
    daplink_blocka_ca7 = curpath.joinpath("./images/daplink_A7.bin")
    daplink_blocka_cm4 = curpath.joinpath("./images/daplink_M4.bin")
    output_path = curpath.joinpath("./images/allinone_upgrade_1M_CRC32.bin")

    # 需包含4字节bin长度+4字节CRC
    input_list = [
                  "{}|{}".format(daplink_blocka_ca7, 700 * 1024),
                  "{}|{}".format(daplink_blocka_cm4, 324 * 1024)
    ]
    packet_bin(output_path, input_list, 6)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
