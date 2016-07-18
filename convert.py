import os
import sys
import re
import time
from unittest import TestCase

WORD_WIDTH = '0xff'
DEFINITION_PADDING = 40
PRODUCTION = False

def production_print(s):
    if PRODUCTION:
        print(s)

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
        production_print('There is no relevant header files in the input directory. Nothing to do..')
        sys.exit(1)
    return files


def get_next_file_content():
    for path in get_file_list():
        production_print('Processing file: {}'.format(path))
        yield (os.path.basename(path), get_file_content(path))


def _parse_version(line, data):
    m = re.search('Version\s+(.+)', line)
    if m:
        version = m.group(1)
        data['version'] = version
        production_print('File version: {}'.format(version))
        return _parse_license
    else:
        return _parse_version


def _parse_license(line, data):
    if 'license' not in data:
        data['license'] = []
    data['license'].append(line)
    if re.search('\*/', line):
        production_print('License parsed')
        return _parse_include_guard
    else:
        return _parse_license


def _parse_include_guard(line, data):
    m = re.search('#ifndef\s+(.+)', line)
    if m:
        guard = m.group(1)
        data['guard'] = guard
        production_print('Include guard: {}'.format(guard))
        production_print('')
        return _parse_include_guard
    m = re.search('#define\s+(.+)', line)
    if m:
        return _parse_reg
    return _parse_include_guard


def _parse_reg(line, data):

    # parse register name
    m = re.search('// Register:\s+(.+)', line)
    if m:
        if 'registers' not in data:
            data['registers'] = []
        reg = m.group(1)
        data['registers'].append({
            'name': reg
        })
        return _parse_reg

    if 'registers' in data and len(data['registers']) > 0:
        last_reg = data['registers'][-1]
    else:
        return _parse_reg

    # parse register address
    m = re.search('extern\s+volatile\s+unsigned\s+char\s+{}\s+@\s+(0x[0-9A-Fa-f]+);'.format(last_reg['name']), line)
    if m:
        address = m.group(1)
        last_reg['address'] = address
        production_print('Register {} @ {}'.format(last_reg['name'], address))
        return _parse_reg

    # parse register section position
    m = re.search('#define\s+_{}_(\w+)_POSITION\s+(0x[0-9A-Fa-f]+)'.format(last_reg['name']), line)
    if m:
        name = m.group(1)
        position = m.group(2)
        if 'sections' not in last_reg:
            last_reg['sections'] = {}
        if name not in last_reg['sections']:
            last_reg['sections'][name] = {}
        last_reg['sections'][name]['position'] = position
        production_print('  {} position: {}'.format(name, position))
        return _parse_reg

    # parse register section size
    m = re.search('#define\s+_{}_(\w+)_SIZE\s+(0x[0-9A-Fa-f]+)'.format(last_reg['name']), line)
    if m:
        name = m.group(1)
        size = m.group(2)
        if 'sections' not in last_reg:
            last_reg['sections'] = {}
        if name not in last_reg['sections']:
            last_reg['sections'][name] = {}
        last_reg['sections'][name]['size'] = size
        production_print('  {} size: {}'.format(name, size))
        return _parse_reg

    # parse register section masks
    m = re.search('#define\s+_{}_(\w+)_MASK\s+(0x[0-9A-Fa-f]+)'.format(last_reg['name']), line)
    if m:
        name = m.group(1)
        value_mask = m.group(2)
        clear_mask = '0x' + format(~int(value_mask, 16) & int(WORD_WIDTH, 16), 'x').upper()
        if 'sections' not in last_reg:
            last_reg['sections'] = {}
        if name not in last_reg['sections']:
            last_reg['sections'][name] = {}
        last_reg['sections'][name]['value-mask'] = value_mask
        last_reg['sections'][name]['clear-mask'] = clear_mask
        production_print('  {} value-mask: {}'.format(name, value_mask))
        production_print('  {} clear-mask: {}'.format(name, clear_mask))
        return _parse_reg

    return _parse_reg


def generate_definition(filename, lines):
    data = {
        'filename': filename
    }
    parser = _parse_version
    for line in lines:
        parser = parser(line, data)
    return data

HEADER = '''\
/* Generated on {date}
===============================================================================
      P I C   T D D   R E A D Y   R E G I S T E R   D E F I N I T I O N
===============================================================================

Created by Tibor Simon - tiborsimon.io
Generator script:  https://github.com/tiborsimon/pic-definition-converter

Based on the register definition header file v{version} provided by Microchip.
Original definition date and license: */

{license}

'''

HEADER_GUARD_START = '''\
#ifndef {guard}
#define {guard}
'''

HEADER_GUARD_STOP = '''\
#endif // {guard}
'''


def write_definition(data):

    lines = []
    date = time.strftime("%Y-%m-%d")
    version = data['version']
    license = '\n'.join(data['license'])
    guard = data['guard']
    lines.append(HEADER.format(date=date, version=version, license=license))
    lines.append(HEADER_GUARD_START.format(guard=guard))
    lines.append('\n')

    for reg in data['registers']:
        if 'sections' in reg:
            lines.append('// Register {}\n'.format(reg['name']))
            for section_name in reg['sections']:
                section = reg['sections'][section_name]
                lines.append('#define {}\n'.format(section_name))
                lines.append('#define {}_address{}{}\n'.format(section_name, ' '*(DEFINITION_PADDING-7-len(section_name)), reg['address']))
                lines.append('#define {}_position{}{}\n'.format(section_name, ' '*(DEFINITION_PADDING-8-len(section_name)), section['position']))
                lines.append('#define {}_size{}{}\n'.format(section_name, ' '*(DEFINITION_PADDING-4-len(section_name)), section['size']))
                lines.append('#define {}_value_mask{}{}\n'.format(section_name, ' '*(DEFINITION_PADDING-10-len(section_name)), section['value-mask']))
                lines.append('#define {}_clear_mask{}{}\n'.format(section_name, ' '*(DEFINITION_PADDING-10-len(section_name)), section['clear-mask']))
                lines.append('\n')


    lines.append(HEADER_GUARD_STOP.format(guard=guard))

    with open(os.path.join('output', data['filename']), 'w+') as f:
        for line in lines:
            f.write(line)

def normalize_definition(definition):
    def rename_section(reg, section_name):
        new_name = reg['name'] + '_' + section_name
        reg['sections'][new_name] = dict(reg['sections'][section_name])
        del reg['sections'][section_name]

    for reg in definition['registers']:
        index = definition['registers'].index(reg)
        if 'sections' in reg:
            for section_name in reg['sections']:
                rename_list = []
                for other_reg in definition['registers'][index+1:]:
                    if 'sections' in other_reg:
                        for other_section_name in other_reg['sections']:
                            if section_name == other_section_name:
                                rename_list.append({
                                    'reg': other_reg,
                                    'section-name': other_section_name
                                })
                if rename_list:
                    rename_section(reg, section_name)
                    for item in rename_list:
                        rename_section(item['reg'], item['section-name'])


if __name__ == '__main__':
    global PRODUCTION
    PRODUCTION = True
    init()
    for filename, lines in get_next_file_content():
        definition = generate_definition(filename, lines)
        normalize_definition(definition)
        write_definition(definition)



#===============================================================================
#  T E S T   S U I T E
#===============================================================================


class DataNormalization(TestCase):
    def test_data_normalization(self):
        data = {
            'registers': [
                {
                    'name': 'REGA',
                    'sections': {
                        'A': {
                            'a': None
                        }
                    }
                },
                {
                    'name': 'REGB',
                    'sections': {
                        'A': {
                            'b': None
                        }

                    }
                }
            ]
        }
        expected = {
            'registers': [
                {
                    'name': 'REGA',
                    'sections': {
                        'REGA_A': {
                            'a': None
                        }
                    }
                },
                {
                    'name': 'REGB',
                    'sections': {
                        'REGB_A': {
                            'b': None
                        }

                    }
                }
            ]
        }
        normalize_definition(data)
        self.assertEquals(expected, data)

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
            'registers': [
                {
                    'name': 'INDF0'
                }
            ]
        }
        expected_next_parser = _parse_reg

        result = _parse_reg(line, data)

        self.assertEqual(expected_next_parser, result)
        self.assertEqual(expected, data)

    def test__parse_register__address_processing(self):
        line = 'extern volatile unsigned char           INDF0               @ 0x0AF;'
        data = {
            'registers': [
                {
                    'name': 'INDF0'
                }
            ]
        }
        expected = {
            'registers': [
                {
                    'name': 'INDF0',
                    'address': '0x0AF'
                }
            ]
        }
        expected_next_parser = _parse_reg

        result = _parse_reg(line, data)

        self.assertEqual(expected_next_parser, result)
        self.assertEqual(expected, data)

    def test__parse_register__section_position(self):
        line = '#define _INDF0_INDF0_POSITION                               0x0'
        data = {
            'registers': [
                {
                    'name': 'INDF0',
                    'address': '0x0AF'
                }
            ]
        }
        expected = {
            'registers': [
                {
                    'name': 'INDF0',
                    'address': '0x0AF',
                    'sections': {
                        'INDF0': {
                            'position': '0x0'
                        }
                    }
                }
            ]
        }
        expected_next_parser = _parse_reg

        result = _parse_reg(line, data)

        self.assertEqual(expected_next_parser, result)
        self.assertEqual(expected, data)

    def test__parse_register__section_size(self):
        line = '#define _INDF0_INDF0_SIZE                                   0x8'
        data = {
            'registers': [
                {
                    'name': 'INDF0',
                    'address': '0x0AF',
                    'sections': {
                        'INDF0': {
                            'position': '0x0'
                        }
                    }
                }
            ]
        }
        expected = {
            'registers': [
                {
                    'name': 'INDF0',
                    'address': '0x0AF',
                    'sections': {
                        'INDF0': {
                            'position': '0x0',
                            'size': '0x8'
                        }
                    }
                }
            ]
        }
        expected_next_parser = _parse_reg

        result = _parse_reg(line, data)

        self.assertEqual(expected_next_parser, result)
        self.assertEqual(expected, data)

    def test__parse_register__section_mask(self):
        line = '#define _INDF0_INDF0_MASK                                   0xF'
        data = {
            'registers': [
                {
                    'name': 'INDF0',
                    'address': '0x0AF',
                    'sections': {
                        'INDF0': {
                            'position': '0x0',
                            'size': '0x8'
                        }
                    }
                }
            ]
        }
        expected = {
            'registers': [
                {
                    'name': 'INDF0',
                    'address': '0x0AF',
                    'sections': {
                        'INDF0': {
                            'position': '0x0',
                            'size': '0x8',
                            'value-mask': '0xF',
                            'clear-mask': '0xF0'
                        }
                    }
                }
            ]
        }
        expected_next_parser = _parse_reg

        result = _parse_reg(line, data)

        self.assertEqual(expected_next_parser, result)
        self.assertEqual(expected, data)

    def test__parse_register_no_mathch(self):
        line = 'helloka'
        data = {}
        expected = {}
        expected_next_parser = _parse_reg

        result = _parse_reg(line, data)

        self.assertEqual(expected_next_parser, result)
        self.assertEqual(expected, data)


