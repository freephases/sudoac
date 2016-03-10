# sudoac
H-Bridge driver for use with LENR Logger<br />
<br />
Produces 1 cycle consisting of 10 segments of equal time:<br />
direction:   [+][0][+][0][0][-][0][-][0][0]<br />
segment:     [0][1][2][3][4][5][6][7][8][9]<br />
  <br />
  wave form  |_|__ _ __<br />
                  | |<br />

<br />
Speed adjustable from ~25Hz to 3.1Hz in a scale of 0 to 100 via serial<br />
<br />
Serial request commands are made up of a command and then data in follow CSV type format:<br />
<br />
[command]|[value|]!<br />
<br />
'!' followed by line return marks end of request command<br />
<br />
Commands:<br />
s   set speed where [value] is 0-100,<br />
    example:  s|100|!<br />
<br />
+   turn on<br />
    example: +|!<br />
<br />
-   turn off<br />
    example: -|!<br />
<br />
Responses can be<br />
<br />
OK|[command]|!<br />
<br />
Command actioned successfully<br />
<br />
Example: OK|s|!<br />
<br />
E|[msg]|!
<br />
Error where [msg] is error message<br />
<br />
Example: E|Error|!<br />

