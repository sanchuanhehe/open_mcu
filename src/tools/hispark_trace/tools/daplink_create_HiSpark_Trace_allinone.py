#!/usr/bin/env python3
# coding=utf-8

'''
# Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2012-2023. All rights reserved.
# daplink_create_HiSpark_Trace_allinone.py Function implementation: Used to generate factory-level chip files.
'''
import struct
import sys
import os
import stat
import pathlib


class Crc16:
    POLYNOMIAL = 0x1021
    PRESET = 0x0000
    _tab = []


    def __init__(self):
        self._tab = [self._initial(i) for i in range(256)]


    def crcb(self, i):
        crc = self.PRESET
        for c in i:
            crc = self._update_crc(crc, c)
        return crc


    def crc(self, string):
        crc = self.PRESET
        for c in string:
            crc = self._update_crc(crc, ord(c))
        return crc


    def _initial(self, c):
        crc = 0
        c = c << 8
        for _ in range(8):
            if (crc ^ c) & 0x8000:
                crc = (crc << 1) ^ self.POLYNOMIAL
            else:
                crc = crc << 1
            c = c << 1
        return crc


    def _update_crc(self, crc, c):
        cc = 0xff & int(c)

        tmp = (crc >> 8) ^ cc
        crc = (crc << 8) ^ self._tab[tmp & 0xff]
        crc = crc & 0xffff

        return crc



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
    #写bin
    data = src.read()
    dst.write(data)

    #写bin对应CRC16
    t = Crc16()
    crc16 = t.crcb(data)
    dst.write(struct.pack('BB', crc16 >> 8, crc16 & 0xFF))

    #剩余部分填充val，val可以为0xFF或者0x0
    packet_fill(dst, pad_len, pad_val)

    #写报文长度, 包含2个字节CRC
    image_len += 2
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
                pad_val = 0 if times != 1 else 0xFFFFFFFF
                packet_bin_write_without_crc(file, subfile, pad_len, pad_val)
                times += 1
            if times >= max_time:
                break
        file.flush()


def main(argv):
    '''
    Function description: Combine loader.bin with image.bin into allinone.bin.
    '''
    curpath = pathlib.Path().cwd()
    bootpath = curpath.joinpath("./images/daplink_boot.stm32")
    parapath = curpath.joinpath("./images/param.bin")
    if os.path.exists(parapath) is False:
        pathlib.Path(parapath).touch()
    daplink_block_a = curpath.joinpath("./images/allinone_upgrade_1M_CRC32.bin")
    daplink_block_b = curpath.joinpath("./images/allinone_upgrade_1M_CRC32.bin")
    output_path = curpath.joinpath("./images/HiSpark-Trace_allinone.bin")

    input_list = [
                  "{}|{}".format(bootpath, 32 * 1024), # Boot partition is 32 * 1024 bytes
                  "{}|{}".format(parapath, 16 * 1024), # Parameter partition is 16 * 1024 bytes
                  "{}|{}".format(daplink_block_a, 1024 * 1024), # A partition is 1024 * 1024 bytes
                  "{}|{}".format(daplink_block_b, 1024 * 1024) # B partition is 1024 * 1024 bytes
    ]
    packet_bin(output_path, input_list, 4)


if __name__ == "__main__":
    sys.exit(main(sys.argv))