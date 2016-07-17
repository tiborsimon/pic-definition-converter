import os
import sys
import unittest
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


if __name__ == '__main__':
    init()
    for data in get_next_data():
        print(data)


class Parsers(TestCase):
    def test__description_persing(self):
        self.assertFalse(True)

