import os
import sys
import re
import time
import copy
try:
    import regart
except ImportError:
    regart = None
from unittest import TestCase


#===============================================================================
#  E N V I R O N M E N T   C O N F I G
#===============================================================================

WORD_WIDTH = '0xFF'
DEFINITION_PADDING = 40
PRODUCTION = False
VERBOSE = False

def production_print(s):
    if PRODUCTION:
        print(s)

def verbose_print(s):
    if VERBOSE:
        print(s)


#===============================================================================
#  I N P U T   F I L E   P A R S I N G
#===============================================================================

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
        yield (os.path.basename(path), get_file_content(path))


def _parse_version(line, data):
    m = re.search('Version\s+(.+)', line)
    if m:
        version = m.group(1)
        data['version'] = version
        verbose_print('File version: {}'.format(version))
        return _parse_license
    else:
        return _parse_version


def _parse_license(line, data):
    if 'license' not in data:
        data['license'] = []
    data['license'].append(line)
    if re.search('\*/', line):
        verbose_print('License parsed')
        return _parse_include_guard
    else:
        return _parse_license


def _parse_include_guard(line, data):
    m = re.search('#ifndef\s+(.+)', line)
    if m:
        guard = m.group(1)
        data['guard'] = guard
        verbose_print('Include guard: {}'.format(guard))
        verbose_print('')
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
        verbose_print('Register {} @ {}'.format(last_reg['name'], address))
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
        verbose_print('  {} position: {}'.format(name, position))
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
        verbose_print('  {} size: {}'.format(name, size))
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
        verbose_print('  {} value-mask: {}'.format(name, value_mask))
        verbose_print('  {} clear-mask: {}'.format(name, clear_mask))
        return _parse_reg

    return _parse_reg


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


def generate_definition(filename, lines):
    data = {
        'filename': filename
    }
    parser = _parse_version
    for line in lines:
        parser = parser(line, data)
    normalize_definition(data)
    return data
    

#===============================================================================
#  O U T P U T   F I L E   G E N E R A T I O N
#===============================================================================

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
    lines_tdd = []

    date = time.strftime("%Y-%m-%d")
    version = data['version']
    license = '\n'.join(data['license'])
    guard = data['guard']
    lines.append(HEADER.format(date=date, version=version, license=license))
    lines.append(HEADER_GUARD_START.format(guard=guard))
    lines.append('\n')
    lines_tdd.append(HEADER.format(date=date, version=version, license=license))
    lines_tdd.append(HEADER_GUARD_START.format(guard=guard))
    lines_tdd.append('\n')

    tdd_data = {'counter': 0}

    for reg in data['registers']:
        if 'sections' in reg:
            _add_register_description(lines, reg)

            section_names = list(reg['sections'].keys())
            sections = []
            
            for section_name in reg['sections']:
                section = reg['sections'][section_name]
                section['name'] = section_name
                sections.append(section)
            sections.sort(key=lambda s: s['position'])

            if reg['name'] not in section_names:
                _add_register_definition(
                    lines,
                    lines_tdd,
                    tdd_data,
                    name=reg['name'],
                    position='0x0',
                    address=reg['address'],
                    size='0x8',
                    value_mask=WORD_WIDTH,
                    clear_mask='0x0'
                )
            for section in sections:
                _add_register_definition(
                    lines,
                    lines_tdd,
                    tdd_data,
                    name=section['name'],
                    position=section['position'],
                    address=reg['address'],
                    size=section['size'],
                    value_mask=section['value-mask'],
                    clear_mask=section['clear-mask']
                )


    lines.append(HEADER_GUARD_STOP.format(guard=guard))
    lines_tdd.append('\n')
    lines_tdd.append(HEADER_GUARD_STOP.format(guard=guard))

    with open(os.path.join('output', data['filename']), 'w+') as f:
        for line in lines:
            f.write(line)

    temp = data['filename'].split('.')
    tdd_filename = temp[0] + '_tdd.' + temp[1]
    with open(os.path.join('output', tdd_filename), 'w+') as f:
        for line in lines_tdd:
            f.write(line)


def _add_register_description(lines, reg):
    if regart:
        reg_in = copy.deepcopy(reg)
        lines.append('\n' + regart.generate(reg_in, forgiveness=True) + '\n')
    else:
        lines.append('// Register {}\n'.format(reg['name']))


def _add_register_definition(lines, lines_tdd, tdd_data, name, position, address, size, value_mask, clear_mask):
    lines.append('#define {}{}{}\n'.format(name, ' '*(DEFINITION_PADDING+1-len(name)), position))
    lines.append('#define {}_address{}{}\n'.format(name, ' '*(DEFINITION_PADDING-7-len(name)), address))
    lines.append('#define {}_position{}{}\n'.format(name, ' '*(DEFINITION_PADDING-8-len(name)), position))
    lines.append('#define {}_size{}{}\n'.format(name, ' '*(DEFINITION_PADDING-4-len(name)), size))
    lines.append('#define {}_value_mask{}{}\n'.format(name, ' '*(DEFINITION_PADDING-10-len(name)), value_mask))
    lines.append('#define {}_clear_mask{}{}\n'.format(name, ' '*(DEFINITION_PADDING-10-len(name)), clear_mask))
    lines.append('\n')

    lines_tdd.append('#define {}{}{}\n'.format(name, ' '*(DEFINITION_PADDING-10-len(name)), tdd_data['counter']))
    tdd_data['counter'] += 1


#===============================================================================
#  M A I N   E N T R Y   P O I N T
#===============================================================================

if __name__ == '__main__':
    # measure execution time
    start = time.clock()

    # input argument parsing
    if len(sys.argv) > 1:
        if sys.argv[1] == '-v':
            VERBOSE = True
            PRODUCTION = False
        else:
            VERBOSE = False
            PRODUCTION = True
    else:
        VERBOSE = False
        PRODUCTION = True

    # init directories
    if not os.path.isdir('input'):
        os.mkdir('input')
    if not os.path.isdir('output'):
        os.mkdir('output')

    # start processing
    file_count = len(get_file_list())
    count = 1
    if file_count == 1:
        print('Processing {} header file..'.format(file_count))
    else:
        print('Processing {} header files..'.format(file_count))
    print('='*80)
    for filename, lines in get_next_file_content():
        print('[{}/{}] {}'.format(count, file_count, filename))
        definition = generate_definition(filename, lines)
        if 'registers' in definition:
            write_definition(definition)
        else:
            print('No parseable content. Nothing to do..')
        count += 1
    print('=' * 80)
    print('Done! Execution took {} seconds.'.format(time.clock() - start))


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


