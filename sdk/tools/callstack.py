#!/usr/bin/env python

import sys

syms = []

def get_symbol(x):
    try:
        addr = int(x, 16)
    except ValueError, e:
        return
    for num in range(len(syms)-1):
        if syms[num][0] <= addr and addr < syms[num+1][0]:
            return '[%08x] ' % (addr) + syms[num][1] + ' + 0x%x' % (addr - syms[num][0])

def main():
    argv = sys.argv
    argc = len(argv)

    if (argc < 3):
        print 'Usage: python %s <System.map> <stackdump.log>' % argv[0]
        quit()

    for line in open(argv[1], 'r'):
        address, type, symbol = line[:-1].split(' ')
        if type == 'T' or type == 't' or type == 'W' or type == 'w':
            syms.append((int(address, 16), symbol))

    callstack = []
    for line in open(argv[2], 'r'):
        print line[:-1]
        if 'up_stackdump:' in line:
            for item in line.split(' '):
                callstack.append(get_symbol(item))

    print '----------------- callstack -----------------'
    for cs in callstack:
        if cs is not None:
            print cs

if __name__ == '__main__':
    main()
