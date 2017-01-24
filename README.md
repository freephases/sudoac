# sudoac
H-Bridge driver for use with LENR Logger 2<br />
<br />
  <br />
Speed adjustable from ~25Hz to 2.1kHz in a scale of 0 to 255 via serial<br />
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

Basic commands are:
<pre> 
  + 
  Power On
  
  - 
  Off
  
  s|[value]! 
  Set speed where value is 0-255, example: s|255

  v|[value]
  Set voltage where value is 0 to 255 where 0=10v, 255=whatever fixedVoltage example:  v|150
  
  w|[wavefrom csv] 
  Set wave form where wavefrom csv is csv string i.e 1,0,2,0 example: w|1,0,2,0
  
  W 
  Display current wave form, output will be W|[wave form csv], i.e: W|1,0,2,0

  ? 
  display handshake, output is OK|go|!


Waveform is composed of:

   1 = (+)(-) normal side on h-bridge
   2 = (-)(+) inverted to "normal" side
   0 = open/off (or anything not a 1 or 2)

   There must be a 0 always before a 1 or a 2. We always start as off (0) so starting with 0 will just delay the start!

    Good Examples:
               _
    1,0,2,0 = | |_   _
                  |_|
              +- 0 -+ 0
                   __
    1,1,0,2,2,0 = |  |_    _
                       |__|
                  +-  0 -+ 0 
           _
    1,0 = | |_
          +- 0
    2,0 =    _
          |_|
          -+ 0
          
   The last 2 will give normal PWM type DC, all the others are AC nd will invert direction of current
   
   Bad examples where x marks a bad cross over are:
                 ___   _
      1,2,0,1 = |   |_| |
                  |_|
                +-xxx0 +-
                     ___
      2,0,1,2 =    _|   |
                |_|   |_|
                -+ 0 +xxx

  No off means certain death for the h-bridge!
    
  1,1,0,2,2,0 is the default wave form, this is longer on (1 or 2) than off (0) type AC sq waveform [+-][+-][0][-+][-+][0]
  
  Others good ones to play with are:
  1,1,1,0,2,2,2,0 - slows freq down, can keep going repeating 1s or 2s
  1,1,0,1,1,0,2,2,0,2,2,0 - maybe closest to wave form that was suggested by someone ;)
  2,2,0,1,0 -- same as 1,1,0,2,0 except it's the other way around!
    
  DC only waveform examples:
  1 - constant +- normal dc
  2 - same as above but someone swiched the wires HA HA!
  1,0 - simple dc pwm
  2,0 - dc pwm but the other way around ;)

</pre> 


