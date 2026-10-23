#!/usr/bin/python3

"""
 @author: Mehmet Kahraman
 @date: 12.03.2024
 @about: Zeroerr joint message class for error and warning messages
"""

class ZeroErrMessages():
    
    def __init__(self):
        self.error_table = {}
        self.warning_table = {}
        self.load_error_messages()
        self.load_warning_messages()
    
    def load_error_messages(self):
        self.error_table = {
            "0x0": "No error",
            "0x2214": "Motor current is over current",
            "0x2250": "Sum of motor three phase current exceed the limit",
            "0x2341": "The U phase over current",
            "0x2342": "The V phase over current",
            "0x2343": "The W phase over current",
            "0x3210": "The bus voltage is overvoltage",
            "0x3220": "The bus voltage is undervoltage",
            "0x4110": "The temperature of the power component is too high",
            "0x7121": "Blocked motor rotation",
            "0x7305": "Encoder initialization failed and abnormal",
            "0x730d": "Battery warning error",
            "0x730f": "Battery low voltage",
            "0x7311": "The position error of the sampled motor end exceeds the limit",
            "0x7314": "The power failure is detected",
            "0x7315": "The sampling load position error exceeds the upper limit",
            "0x7374": "Multi-turn position error",
            "0x8400": "The velocity error exceeds the limit value",
            "0x8401": "Motor velocity exceeds the limit value",
            "0x8500": "The position error exceeds the limit value",
            "0xa000": "The master station goes offline",
            "0xf004": "EtherCAT initialization error",
            "0xf005": "The STO function is activated",
            "0xf006": "Multi-turn circle count error"
        }
    
    def load_warning_messages(self):
        self.warning_table = {
            "0x0": "No warning",
        }
    
    def get_error_message(self, error_code):
        if error_code in self.error_table:
            return self.error_table[error_code]
        else:
            return "Unknown error code: " + error_code
    
    def get_warning_message(self, warn_code):
        if warn_code in self.warning_table:
            return self.error_table[warn_code]
        else:
            return "Unknown warning code: " + warn_code