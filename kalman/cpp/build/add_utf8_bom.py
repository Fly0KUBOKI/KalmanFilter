#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import glob

def add_utf8_bom_to_file(filepath):
    """Add UTF-8 BOM to a file if it doesn't already have one"""
    try:
        with open(filepath, 'rb') as f:
            content = f.read()
        
        # Check if file already has BOM
        if content.startswith(b'\xef\xbb\xbf'):
            return False  # Already has BOM
        
        # Add BOM and write back
        with open(filepath, 'wb') as f:
            f.write(b'\xef\xbb\xbf' + content)
        
        return True
        
    except Exception as e:
        print(f'Error: {filepath}: {e}')
        return False

def main():
    base_dirs = [
        r'c:\Users\takut\OneDrive\ドキュメント\MATLAB\KalmanFilter\kalman\cpp\Inc',
        r'c:\Users\takut\OneDrive\ドキュメント\MATLAB\KalmanFilter\kalman\cpp\MEX',
        r'c:\Users\takut\OneDrive\ドキュメント\MATLAB\KalmanFilter\kalman\cpp\Src',
    ]
    
    count = 0
    for base_dir in base_dirs:
        # Find all C++ header/source files
        patterns = [
            os.path.join(base_dir, '**', '*.hpp'),
            os.path.join(base_dir, '**', '*.cpp'),
            os.path.join(base_dir, '*.hpp'),
            os.path.join(base_dir, '*.cpp'),
        ]
        
        for pattern in patterns:
            files = glob.glob(pattern, recursive=True)
            for f in files:
                if add_utf8_bom_to_file(f):
                    print(f'Added BOM: {os.path.basename(f)}')
                    count += 1
    
    print(f'\nTotal files with BOM added: {count}')

if __name__ == '__main__':
    main()
