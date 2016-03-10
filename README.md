# sudoac
H-Bridge driver for use with LENR Logger

Produces 1 cycle consisting of 10 segments of equal time:
direction:   [+][0][+][0][0][-][0][-][0][0]
segment:     [0][1][2][3][4][5][6][7][8][9]
  
  wave form  |_|__ _ __
                  | |

Speed adjustable from ~25Hz to 3.1Hz in a scale of 0 to 100 via serial

Serial request commands are made up of a command and then data in follow CSV type format:

[command]|[value|]!

'!' followed by line return marks end of request command

Commands:
s   set speed where [value] is 0-100,
    example:  s|100|!

+   turn on
    example: +|!

-   turn off
    example: -|!

Responses can be

OK|[command]|!

Command actioned successfully

Example: OK|s|!

E|[msg]|!

Error where [msg] is error message

Example: E|Error|!

