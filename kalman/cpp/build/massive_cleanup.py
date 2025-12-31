#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import glob

def remove_japanese_from_file(filepath):
    """Remove all Japanese characters from a file"""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Remove Japanese characters (but keep comments)
        # Replace Japanese with empty string
        content = re.sub(r'[\u3040-\u309F\u30A0-\u30FF\u4E00-\u9FFF\u3400-\u4DBF]', '', content)
        
        # Write back
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(content)
        
        return True
        
    except Exception as e:
        print(f'Error: {filepath}: {e}')
        return False

def main():
    base_dir = r'c:\Users\takut\OneDrive\ドキュメント\MATLAB\KalmanFilter\kalman\cpp\Inc'
    
    # Find all header files
    patterns = [
        os.path.join(base_dir, '**', '*.hpp'),
        os.path.join(base_dir, '..', 'MEX', '*.hpp'),
        os.path.join(base_dir, '..', 'MEX', '*.cpp'),
        os.path.join(base_dir, '..', 'Src', '**', '*.hpp'),
        os.path.join(base_dir, '..', 'Src', '**', '*.cpp'),
    ]
    
    count = 0
    for pattern in patterns:
        files = glob.glob(pattern, recursive=True)
        for f in files:
            if remove_japanese_from_file(f):
                print(f'Cleaned: {os.path.basename(f)}')
                count += 1
    
    print(f'\nTotal files cleaned: {count}')

if __name__ == '__main__':
    main()
