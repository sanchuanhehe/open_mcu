#!/usr/bin/env python
# coding=utf-8

'''
# Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2012-2023. All rights reserved.
# daplink_create_loader.py Function implementation: Packs boot binary files.
'''
import struct
import sys
import os
import stat
import pathlib


_MAGIC_NUMBER = 0xAA55A55A
_HIC_ID = 0
_FILE_TYPE = 0x9B939D93
_VER = 0
_FACTORY_ID = 0
_CHIP_ID = 0
_FLASH_TYPE = 0


def packet_fill(dst, length, val):
    while length >= 32:
        dst.write(struct.pack('IIIIIIII',
            val, val, val, val, val, val, val, val))
        length = length - 32
    while length >= 4:
        dst.write(struct.pack('I', val))
        length = length - 4
    while length > 0:
        dst.write(struct.pack('B', val & 0xFF))
        length = length - 1


def add_loader_header(file_len):
    header = []
    header.append(_MAGIC_NUMBER)
    header.append(_HIC_ID)
    header.append(_FILE_TYPE)
    header.append(_VER)
    header.append(_FACTORY_ID)
    header.append(_CHIP_ID)
    header.append(_FLASH_TYPE)
    header.append(file_len)
    for _i in range(4):
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
    
    header = add_loader_header(int(burn_size))
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


def image_header_checksum(data, offset, length):
    csum = 0
    for i in range(offset, length):
        csum += data[i]
    return csum


def write_info_to_data(data_in, data_out, offset):
    for i in range(4):
        data_out[offset + i] = data_in.to_bytes(length=4, byteorder='big')[3 - i]


def gen_stm32(input_path, output_path):
    stm32_hdr_offset = {
        "_MAGIC_NUMBER" : 0,
        "image_signature" : 4,
        "image_checksum" : 68,
        "header__VERsion" : 72,
        "image_length" : 76,
        "image_entry_point" : 80,
        "load_address" : 88,
        "_VERsion_number" : 96,
        "option_flags" : 100,
        "ecdsa_algorithm" : 104,
        "ecdsa_public_key" : 108,
        "binary_type" : 252,
    }
    flag = 0xefbeaddf
    modes = stat.S_IWUSR | stat.S_IRUSR
    flags = os.O_RDWR | os.O_CREAT
    data = [0] * 256
    with os.fdopen(os.open(input_path, flags, modes), 'rb+') as subfile:
        data += subfile.read()
        write_info_to_data(0x324D5453, data, stm32_hdr_offset.get('_MAGIC_NUMBER'))
        write_info_to_data(0x00010000, data, stm32_hdr_offset.get('header__VERsion'))
        image_len = os.path.getsize(input_path)
        write_info_to_data(image_len, data, stm32_hdr_offset.get('image_length'))
        write_info_to_data(0x2ffc2500, data, stm32_hdr_offset.get('image_entry_point'))
        write_info_to_data(0x2ffc2500, data, stm32_hdr_offset.get('load_address'))
        write_info_to_data(0x1, data, stm32_hdr_offset.get('option_flags'))
        write_info_to_data(0x1, data, stm32_hdr_offset.get('ecdsa_algorithm'))
        image_checksum = image_header_checksum(data, 256, image_len + 256)
        write_info_to_data(image_checksum, data, stm32_hdr_offset.get('image_checksum'))

    with os.fdopen(os.open(output_path, flags, modes), 'wb+') as file:
        file.write(bytes(data))


def main(argv):
    '''
    Function description: Combine loader.bin with image.bin into allinone.bin.
    '''
    gen_stm32('images/daplink_boot.bin', 'images/daplink_boot.stm32')
    curpath = pathlib.Path().cwd()
    bootpath = curpath.joinpath("images/daplink_boot.stm32")
    output_path = curpath.joinpath("images/HiSpark-Trace_Boot_CRC32_forDAPLINK.bin")

    input_list = ["{}|{}".format(bootpath, 32 * 1024)]
    packet_bin(output_path, input_list)

if __name__ == "__main__":
    sys.exit(main(sys.argv))
