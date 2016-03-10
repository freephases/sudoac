# sudoac
H-Bridge driver for use with LENR Logger<br />
<br />
Produces 1 cycle consisting of 10 segments of equal time:<br />
direction:   [+][0][+][0][0][-][0][-][0][0]<br />
segment:     [0][1][2][3][4][5][6][7][8][9]<br />
  <br />
<pre>
  wave form  |_|__ _ __
                  | |
</pre>
<br />
Speed adjustable from ~25Hz to 3.1kHz in a scale of 0 to 100 via serial<br />
<br />
Serial request commands are made up of a command and then data in follow CSV type format:<br />
<br />
[command]|[value|]!<br />
<br />
'!' followed by line return marks end of request command<br />
<br />
Commands:<br />
<strong>s</strong>   set speed where [value] is 0-100,<br />
    example:  s|100|!<br />
<br />
<strong>+</strong>   turn on<br />
    example: +|!<br />
<br />
<strong>-</strong>   turn off<br />
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

