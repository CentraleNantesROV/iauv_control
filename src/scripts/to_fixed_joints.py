#!/usr/bin/env python
import sys
import xml.etree.ElementTree as ET
import argparse

def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.description = 'Changes moving joints to fixed ones for embedded use'
    parser.add_argument('-d','--description', type=str, help='Raw robot description')
    parser.add_argument('--to_fixed',type=str, nargs='+',help='Frame prefix', default=[])
    parser.add_argument('--keep_moving', nargs='+', type=str, default=[])
    
    args = parser.parse_args()    
        
    xml = ET.fromstring(args.description)
    
    only_some = len(args.to_fixed) != 0
    def should_be_fixed(name):
        if (only_some and name in args.to_fixed) or not only_some:
            return name not in args.keep_moving
    
    for joint in xml.iter('joint'):
        if should_be_fixed(joint.attrib['name']):
            joint.attrib['type'] = 'fixed'
    
    # print to stdout and let SimpleLauncher pass it to robot_state_publisher
    ET.dump(xml)
            
if __name__ == "__main__":
    main()
 
