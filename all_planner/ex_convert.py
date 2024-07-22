#!/usr/bin/env python3

def convert_normal_to_pos_w(pos):
    return (pos[0]*3 - 16.5, pos[1]*3 - 28.5)

def convert_normal_to_pos_p(pos):
    return (pos[0]*3 - 7.5, pos[1]*3 - 67.5)

def convert_pos_to_normal_w(normal):
    return (int((normal[0] + 16.5) / 3), int((normal[1] + 28.5) / 3))

def convert_pos_to_normal_p(normal):
    return (int((normal[0] + 7.5) / 3), int((normal[1] + 67.5) / 3))