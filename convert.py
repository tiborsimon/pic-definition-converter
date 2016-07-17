import os
import sys
import re
from unittest import TestCase


def init():
    if not os.path.isdir('input'):
        os.mkdir('input')
    if not os.path.isdir('output'):
        os.mkdir('output')


def get_file_content(path):
    with open(path) as f:
        ret = []
        for line in f.readlines():
            ret.append(line.strip())
        return ret


def get_file_list():
    files = os.listdir('input')
    files = [os.path.join(os.getcwd(), 'input', file) for file in files if file.endswith('.h')]
    if not files:
        print('There is no relevant header files in the input directory. Nothing to do..')
        sys.exit(1)
    return files


def get_next_data():
    for path in get_file_list():
        yield get_file_content(path)


def _parse_version(line, data):
    m = re.search('Version\s+(.+)', line)
    if m:
        data['version'] = m.group(1)
        return _parse_license
    else:
        return _parse_version


def _parse_license(line, data):
    if 'license' not in data:
        data['license'] = []
    data['license'].append(line)
    if re.search('\*/', line):
        return _parse_include_guard
    else:
        return _parse_license


def _parse_include_guard(line, data):
    m = re.search('#ifndef\s+(.+)', line)
    if m:
        data['guard'] = m.group(1)
        return _parse_include_guard
    m = re.search('#define\s+(.+)', line)
    if m:
        return _parse_reg
    return _parse_include_guard


def _parse_reg(line, data):
    if 'registers' not in data:
        data['registers'] = {}
    m = re.search('// Register:\s+(.+)', line)
    if m:
        reg = m.group(1)
        data['registers'][reg] = {
            'done': False
        }
        return _parse_reg





def generate_definition(data):
    pass


def write_definition(data):
    pass


if __name__ == '__main__':
    init()
    for data in get_next_data():
        definition = generate_definition(data)
        write_definition(data)


class Parsers(TestCase):
    def test_version_parser_on_match(self):
        line = '// Version 1.37'
        data = {}
        expected = {
            'version': '1.37'
        }
        expected_next_parser = _parse_license

        result = _parse_version(line, data)

        self.assertEqual(expected_next_parser, result)
        self.assertEqual(expected, data)

    def test_version_parser_on_no_match(self):
        line = '// something'
        data = {}
        expected = {}
        expected_next_parser = _parse_version

        result = _parse_version(line, data)

        self.assertEqual(expected_next_parser, result)
        self.assertEqual(expected, data)

    def test__license_parser_on_match(self):
        line = '*  */'
        data = {
            'license': [
                'something'
            ]
        }
        expected = {
            'license': [
                'something',
                '*  */'
            ]
        }
        expected_next_parser = _parse_include_guard

        result = _parse_license(line, data)

        self.assertEqual(expected_next_parser, result)
        self.assertEqual(expected, data)

    def test__license_parser_on_no_match(self):
        line = 'something'
        data = {}
        expected = {
            'license': [
                'something'
            ]
        }
        expected_next_parser = _parse_license

        result = _parse_license(line, data)

        self.assertEqual(expected_next_parser, result)
        self.assertEqual(expected, data)

    def test__parse_include_guards__ifndef(self):
        line = '#ifndef _PIC16F1454_H_'
        data = {}
        expected = {
            'guard': '_PIC16F1454_H_'
        }
        expected_next_parser = _parse_include_guard

        result = _parse_include_guard(line, data)

        self.assertEqual(expected_next_parser, result)
        self.assertEqual(expected, data)

    def test__parse_include_guards__define(self):
        line = '#define _PIC16F1454_H_'
        data = {
            'guard': '_PIC16F1454_H_'
        }
        expected = {
            'guard': '_PIC16F1454_H_'
        }
        expected_next_parser = _parse_reg

        result = _parse_include_guard(line, data)

        self.assertEqual(expected_next_parser, result)
        self.assertEqual(expected, data)

    def test__parse_register__start_line(self):
        line = '// Register: INDF0'
        data = {}
        expected = {
            'registers': {
                'INDF0': {
                    'done': False
                }
            }
        }
        expected_next_parser = _parse_reg

        result = _parse_reg(line, data)

        self.assertEqual(expected_next_parser, result)
        self.assertEqual(expected, data)

#     def test__description_parsing_lk(self):
#         lines = '''\
# // Version 1.37
# // Generated 11/03/2016 GMT
#
# /*
#  * Copyright © 2016, Microchip Technology Inc. and its subsidiaries ("Microchip")
#  * All rights reserved.
#  *
#  * This software is developed by Microchip Technology Inc. and its subsidiaries ("Microchip").
#  *
#  * Redistribution and use in source and binary forms, with or without modification, are
#  * permitted provided that the following conditions are met:
#  *
#  *     1. Redistributions of source code must retain the above copyright notice, this list of
#  *        conditions and the following disclaimer.
#  *
#  *     2. Redistributions in binary form must reproduce the above copyright notice, this list
#  *        of conditions and the following disclaimer in the documentation and/or other
#  *        materials provided with the distribution.
#  *
#  *     3. Microchip's name may not be used to endorse or promote products derived from this
#  *        software without specific prior written permission.
#  *
#  * THIS SOFTWARE IS PROVIDED BY MICROCHIP "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
#  * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL MICROCHIP BE LIABLE FOR ANY DIRECT, INDIRECT,
#  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING BUT NOT LIMITED TO
#  * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA OR PROFITS; OR BUSINESS
#  * INTERRUPTION) HOWSOEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
#  * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#  *  */'''
#         lines = lines.split('\n')
#         expected = dict(lines) + ['', '']
#         result = _parse_description(lines)

