# !/usr/bin/env python
# -*- coding: utf-8 -*-

# @copyright Copyright (c) 2022, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
# following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
# disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
# following disclaimer in the documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
# products derived from this software without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
# USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ci_entry.py Function implementation: CI build entry file, which is used to 
# copy code and invoke build compilation scripts.

import os
import sys
import pathlib
from configparser import ConfigParser
import stat
import shutil
import shlex
import subprocess
import platform

from build_gn import read_json_file
from ide_entry import un_alltools
from build import remove_readonly

# ci release map
chose_ci_release_sdk = {
    "SolarA2_1.0.0": "3065h",
    "SolarA2_1.0.1": "3061m",
    "SolarA2_1.0.2": "3015",
    "SolarA2_1.1.0": "3066m",
    "SolarA2_1.1.1": "3066h"
}


def dest_path_cpoy(copy_path, dirprocesspath, dirpath, filenames):
    '''
    Function description: Source file copy to destination address.
    '''
    # Destination address
    dest_path = pathlib.Path(copy_path).joinpath(dirprocesspath)
    if not dest_path.exists():
        os.makedirs(dest_path)
    # Copy files  
    for file in filenames:
        if 'entry.py' in file or 'trustlist.json' in file:
            continue
        source_file = pathlib.Path(dirpath).joinpath(file)
        shutil.copy(source_file, dest_path)


def filter_copy_adpater(copy_path, dirpath, filenames):
    '''
    Function description: project create.
    '''
    ipbefore_list = []
    ipafter_list = []
    cur_sys = platform.system()
    if cur_sys == 'Windows':
        ipbefore_list = dirpath.split('\\')
    elif cur_sys == 'Linux':
        ipbefore_list = dirpath.split('/')
    ipafter_list = []
    p_vindex = 0
    p_cindex = -1
    for v_index, ipdir in enumerate(ipbefore_list):
        if ipdir.startswith('v') and '.' in ipdir:
            continue
        # Filtering files
        if '_v' in ipdir:
            p_vindex = v_index
        if 'common_v' in ipdir:
            p_cindex = v_index
        ipafter_list.append(ipdir)
    # Filtering similar catalogs with v
    if ipafter_list[p_vindex - 1] + '_v' in ipafter_list[p_vindex]:
        ipafter_list.pop(p_vindex)
    if p_cindex >= 0:
        ipafter_list[p_cindex] = 'common'
        p_cindex = -1
    dirprocesspath = '/'.join(ipafter_list)
    dest_path_cpoy(copy_path, dirprocesspath, dirpath, filenames)


def traversal_path(copy_path, module_path, version):
    '''
    Function description: create and copy work.
    '''
    # copy file by different chip version
    for (dirpath, _, filenames) in os.walk(module_path):
        filter_copy_adpater(copy_path, dirpath, filenames)


def copy_drive_modules(copy_abspath, product, ip_name, copyjson_content):
    '''
    Function description: Copying drive modules
    '''
    source_path = os.path.join('drivers', ip_name)
    for ip_content in copyjson_content[ip_name]:
        # Merge paths
        source_path = os.path.join(source_path, ip_content)
        if pathlib.Path(source_path).is_dir():
            traversal_path(copy_abspath, source_path, product)
        # File copy processing in driver module
        elif pathlib.Path(source_path).is_file():
            parent_path = pathlib.Path(source_path).parent
            copy_parent_path = pathlib.Path(copy_abspath).joinpath(parent_path)
            # Create if the file does not exist
            if not copy_parent_path.exists():
                os.makedirs(copy_parent_path)
            shutil.copy(source_path, copy_parent_path)
        source_path = ''
        source_path = os.path.join('drivers', ip_name)


def copy_code(product, copy_path):
    '''
    Function description: file filter for different chip type
    '''
    # Instantiation parameter check.
    if not isinstance(copy_path, str):
        raise TypeError("copy_path in para type error {}".format(
                        type(copy_path)))
    if not isinstance(product, str):
        raise TypeError("product in para type error {}".format(
                        type(product)))
    # Copy file path
    if pathlib.Path(copy_path).exists():
        shutil.rmtree(os.path.realpath(copy_path), onerror=remove_readonly)
    copy_abspath = pathlib.Path(copy_path).resolve()
    copyjson_path = pathlib.Path.cwd()\
                    .joinpath('chip', "{}".format(product), 'codecopy.json')
    copyjson_content = read_json_file(copyjson_path)
    # Add the basic modules
    for module_path in copyjson_content['modules']:
        if pathlib.Path(module_path).is_dir():
            traversal_path(copy_abspath, module_path, product)
        elif pathlib.Path(module_path).is_file():
            # File copy processing in basic module
            parent_path = pathlib.Path(module_path).parent
            copy_parent_path = pathlib.Path(copy_abspath).joinpath(parent_path)
            # Create if the file does not exist
            if not copy_parent_path.exists():
                os.makedirs(copy_parent_path)
            shutil.copy(module_path, copy_parent_path)
            if str(parent_path) == 'chip\\target' or str(parent_path) == 'chip/target' :
                userconfig_json_name = module_path.split('/')[-1]
                os.rename(pathlib.Path(copy_parent_path).joinpath(userconfig_json_name),
                    pathlib.Path(copy_parent_path).joinpath('userconfig.json'))
    # Copying drive modules
    for ip_name in copyjson_content['ip_drive_file']:
        copy_drive_modules(copy_abspath, product, ip_name, copyjson_content)


def differ_file_copy(copy_chip, copy_path, tools_path):
    '''
    Function description: Projection generation for different chip
    '''
    # Decompress all compilation tools.
    un_alltools(tools_path)

    # Detach the chip package.
    copy_code(copy_chip, copy_path)
    os.chdir(pathlib.Path(copy_path).resolve())
    target_pathconfigsource = pathlib.Path().cwd().joinpath('chip', copy_chip, 'target')
    target_pathconfigdest = pathlib.Path().cwd().joinpath('chip', 'target')
    # Target file replace
    if pathlib.Path(target_pathconfigsource).exists():
        shutil.rmtree(target_pathconfigdest)
        shutil.copytree(target_pathconfigsource, target_pathconfigdest)
    # Write the path of the full package tool to the config.ini file.
    config_path = pathlib.Path().cwd().joinpath('build', 'config.ini')
    config = ConfigParser()
    config.read(config_path)
    config.set('gn_args', 'tools_path', tools_path)
    flags = os.O_WRONLY | os.O_CREAT | os.O_TRUNC
    modes = stat.S_IWUSR | stat.S_IRUSR
    flagsone = os.O_RDWR | os.O_CREAT
    # File socket of the config.ini file.
    with os.fdopen(os.open(config_path, flags, modes), 'w+') as configini:
        config.write(configini)
    # Executing Build and Compile Commands
    cmd = shlex.split('python build/build.py build')
    proc = subprocess.Popen(cmd)
    proc.wait()
    ret_code = proc.returncode
    if ret_code != 0:
        raise Exception("CI {} failed, return code is {}".format(cmd, ret_code))


def ci_entry(argv):
    '''
    Function description: ci entry function.
    '''
    if not pathlib.Path('build/ci_entry.py').exists():
        os.chdir("/home/ci/driver")
    # Save the path of the full package tool.
    tools_path = str(pathlib.Path().cwd().joinpath('tools', 'toolchain'))
    # CI building or releasing
    if len(argv) == 3:
        if argv[0] == 'release':
            copy_chip = argv[1]
            copy_path = argv[2]
        else:
            copy_chip = argv[1] + '_' + argv[2]
            # pick chip name from map
            try:
                copy_chip = chose_ci_release_sdk[copy_chip]
            except KeyError:
                copy_chip = '3065h'
            copy_path = 'mcu_' + copy_chip + '_project'
    else:
        # default value
        copy_chip = '3065h'
        copy_path = '../mcu_pro'
    # Copying and generating project files
    working_path = pathlib.Path.cwd()
    differ_file_copy(copy_chip, copy_path, tools_path)

if __name__ == "__main__":
    sys.exit(ci_entry(sys.argv))
