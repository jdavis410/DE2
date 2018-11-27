; This program includes...
; - Robot initialization (checking the battery, stopping motors, etc.).
; - The movement API.
; - Several useful subroutines (ATAN2, Neg, Abs, mult, div).
; - Some useful constants (masks, numbers, robot stuff, etc.)

; This code uses the timer interrupt for the movement control code.
; The ISR jump table is located in mem 0-4.  See manual for details.
ORG 0
	JUMP   Init        ; Reset vector
	RETI               ; Sonar interrupt (unused)
	JUMP   CTimer_ISR  ; Timer interrupt
	RETI               ; UART interrupt (unused)
	RETI               ; Motor stall interrupt (unused)

;***************************************************************
;* Initialization
;***************************************************************
Init:
	; Always a good idea to make sure the robot
	; stops in the event of a reset.
	LOAD   Zero
	OUT    LVELCMD     ; Stop motors
	OUT    RVELCMD
	STORE  DVel        ; Reset API variables
	STORE  DTheta
	OUT    SONAREN     ; Disable sonar (optional)
	OUT    BEEP        ; Stop any beeping (optional)
	
	CALL   SetupI2C    ; Configure the I2C to read the battery voltage
	CALL   BattCheck   ; Get battery voltage (and end if too low).
	OUT    LCD         ; Display battery voltage (hex, tenths of volts)
	
WaitForSafety:
	; This loop will wait for the user to toggle SW17.  Note that
	; SCOMP does not have direct access to SW17; it only has access
	; to the SAFETY signal contained in XIO.
	IN     XIO         ; XIO contains SAFETY signal
	AND    Mask4       ; SAFETY signal is bit 4
	JPOS   WaitForUser ; If ready, jump to wait for PB3
	IN     TIMER       ; We'll use the timer value to
	AND    Mask1       ;  blink LED17 as a reminder to toggle SW17
	SHIFT  8           ; Shift over to LED17
	OUT    XLEDS       ; LED17 blinks at 2.5Hz (10Hz/4)
	JUMP   WaitForSafety
	
WaitForUser:
	; This loop will wait for the user to press PB3, to ensure that
	; they have a chance to prepare for any movement in the main code.
	IN     TIMER       ; We'll blink the LEDs above PB3
	AND    Mask1
	SHIFT  5           ; Both LEDG6 and LEDG7
	STORE  Temp        ; (overkill, but looks nice)
	SHIFT  1
	OR     Temp
	OUT    XLEDS
	IN     XIO         ; XIO contains KEYs
	AND    Mask2       ; KEY3 mask (KEY0 is reset and can't be read)
	JPOS   WaitForUser ; not ready (KEYs are active-low, hence JPOS)
	LOAD   Zero
	OUT    XLEDS       ; clear LEDs once ready to continue


;***************************************************************
;* Main code
;***************************************************************
Main:
	OUT    RESETPOS    ; reset the odometry to 0,0,0
	; configure timer interrupt for the movement control code
	LOADI  10          ; period = (10 ms * 10) = 0.1s, or 10Hz.
	OUT    CTIMER      ; turn on timer peripheral
	
	;;;; Demo code to acquire sonar data during a rotation
	CLI    &B0010      ; disable the movement API interrupt
	CALL   AcquireData ; perform a 360 degree scan
	
	;;;; Demo code to turn to face the closest object seen
	; Before enabling the movement control code, set it to
	; not start moving immediately.
	LOADI  0
	STORE  DVel        ; zero desired forward velocity
	IN     THETA
	STORE  DTheta      ; desired heading = current heading
	SEI    &B0010      ; enable interrupts from source 2 (timer)
	; at this point, timer interrupts will be firing at 10Hz, and
	; code in that ISR will attempt to control the robot.
	; If you want to take manual control of the robot,
	; execute CLI &B0010 to disable the timer interrupt.

	; FindClosest returns the angle to the closest object
	CALL 	wallFinder

InfLoop: 
	JUMP   InfLoop
	; note that the movement API will still be running during this
	; infinite loop, because it uses the timer interrupt.
	
	

; AcquireData will turn the robot counterclockwise and record
; 360 sonar values in memory.  The movement API must be disabled
; before calling this subroutine.
AcquireData:
	; Get the current angle so we can stop after one circle.
	IN     THETA
	STORE  OrigTheta
	STORE  CurrTheta
	; Record that a rotation is just starting
	LOAD   Zero
	STORE  TurnTracker
	; turn on sonar 0
	LOAD   Mask0
	OUT    SONAREN
	; stop the left motor
	LOAD   Zero
	OUT    LVELCMD
		
ADWait:
	; turn the robot, using the right wheel only
	LOAD   FSlow
	OUT    RVELCMD
	; wait until turned to a new angle
	IN     Theta
	XOR    CurrTheta
	JZERO  ADWait      ; same angle; wait until turned more
	
	; Check if a significant turn has occurred.  This is for
	; robustness, so that a small clockwise turn at the start
	; doesn't immediately exit the routine.
	IN     Theta
	SUB    OrigTheta
	CALL   Abs
	ADDI   180         ; account for angle wrapping
	CALL   Mod360
	ADDI   -180
	CALL   Abs
	ADDI   -10         ; 10 degree margin
	JNEG   ADStore     ; margin not passed
	LOADI  1           ; margin passed
	STORE  TurnTracker
	
ADStore:
	; store a data point
	IN     THETA
	STORE  CurrTheta   ; update current angle
	ADDI   90          ; since this sonar is facing left
	CALL   Mod360      ; wrap angles >360
	ADDI   DataArray   ; index into the array
	STORE  ArrayIndex
	IN     DIST0
	ISTORE ArrayIndex  ; store this data point
	
	; check if we've gone 360
	LOAD   TurnTracker
	JZERO  ADWait      ; haven't turned at all
	IN     THETA
	XOR    OrigTheta
	JPOS   ADWait
	JNEG   ADWait
	RETURN ; done
	
	ArrayIndex: DW 0
	OrigTheta: DW 0
	CurrTheta: DW 0
	TurnTracker: DW 0
	
Wait4User: 
	STORE CurrentAC
	
Wait4UserLoop:
	LOAD CurrentAC
	OUT    SSEG2
	LOAD W4ULEDS
	OUT LEDS
	IN     TIMER       ; We'll blink the LEDs above PB3
	AND    Mask1
	SHIFT  5           ; Both LEDG6 and LEDG7
	STORE  Temp        ; (overkill, but looks nice)
	SHIFT  1
	OR     Temp
	OUT    XLEDS
	IN     XIO         ; XIO contains KEYs
	AND    Mask2       ; KEY3 mask (KEY0 is reset and can't be read)
	JPOS   Wait4UserLoop ; not ready (KEYs are active-low, hence JPOS)
	LOAD   Zero
	OUT    XLEDS       ; clear LEDs once ready to continue
W4U2:
	IN     XIO         ; XIO contains KEYs
	AND    Mask2       ; KEY3 mask (KEY0 is reset and can't be read)
	JZERO   W4U2 	   ; not ready (KEYs are active-low, hence JPOS)
	
	LOAD CurrentAC
	RETURN
CurrentAC: DW 0
W4ULEDS:   DW 0
	
wallFinder: 	LOADI DataArray			; load address of DataArray
				STORE WFArrayIndex        ; Store address of DataArray into ArrayIndex 
				;LOADI 0
				;STORE WFCurrentTheta
				;STORE WFNumOfWalls
				;LOADI -1
				;STORE DACurrentWall

WFLoop1: 		LOAD WFCurrentTheta		; Load WFCurrentTheta
				ADDI -359				;check if done
				JPOS findWallsDA			;to change
				ILOAD WFArrayIndex		; Load mem(mem(arrayIndex))
				STORE WFCurrentValue 	;get the sonar reading for the current angle
				SUB WFInvalid 			;check if value is invalid
				JNEG WFNotInvalid
				LOAD WFArrayIndex			;increment array index and angle value
				ADDI 1
				STORE WFArrayIndex
				LOAD WFCurrentTheta		; increment WFCurrentTheta
				ADDI 1
				
				STORE WFCurrentTheta
				JUMP WFLoop1            ;back to loop

WFNotInvalid:	LOAD WFCurrentValue 	
				SHIFT -3		;CurrentMax = sonar value shifted right three bits + WFCurrentValue
				ADD WFCurrentValue		;      (i.e. 1/8 = .125 is slightly above the error margin around most walls)
				STORE WFCurrentMax
				LOAD WFCurrentValue
				SHIFT -3
				XOR WFABunchOfOnes
				ADDI 1
				ADD WFCurrentValue
				STORE WFCurrentMin		;currentMin = CurrentValue - value shifted right 3 bits
				LOADI 1
				STORE WFDeltaTheta

WFLoop2:		LOADI DataArray
				STORE WFArrayIndex       	
				LOAD WFCurrentTheta
				ADD WFDeltaTheta
				CALL Mod360
				ADD WFArrayIndex
				STORE WFArrayIndex 		;ArrayIndex now refers to the next value to check
				ILOAD WFArrayIndex
				STORE WFValueToTest
				SUB WFCurrentMin        ;compare with minimum allowable value
				JNEG WFEndOfWall
				LOAD WFValueToTest     
				SUB WFCurrentMax        ;compare with maximum allowable value
				JPOS WFEndOfWall
				LOAD WFDeltaTheta
				ADDI 1
				STORE WFDeltaTheta		;increment DeltaTheta
				JUMP WFLoop2            ;keep loopin'

WFEndOfWall:    LOADI 1
				STORE W4ULEDS
				LOAD WFDeltaTheta       
				ADDI -15                ;compare with the required wall length
				JNEG WFNotAWall
				LOADI FoundWalls
				ADD WFNumOfWalls
				STORE WFArrayIndex        ;arrayIndex refers to location to store next wall position (in the found walls array)
				LOAD WFDeltaTheta
				SHIFT -1      ;add half of wall length to currentTheta to get center of wall
				ADD WFCurrentTheta
				CALL Wait4User
				ISTORE WFArrayIndex
				LOAD WFNumOfWalls
				ADDI 1
				STORE WFNumOfWalls
				LOAD WFCurrentTheta
				ADD WFDeltaTheta 	
				STORE WFCurrentTheta 	;update CurrentTheta
				ADDI DataArray
				STORE WFArrayIndex 		;update ArrayIndex
				JUMP WFLoop1

WFNotAWall:		LOAD WFCurrentTheta
				ADDI 1
				STORE WFCurrentTheta 	;increment CurrentTheta
				ADDI DataArray
				STORE WFArrayIndex 		;update ArrayIndex
				JUMP WFLoop1

WFCurrentTheta:	DW 0
WFValueToTest:  DW 0
WFDeltaTheta:	DW 0
WFCurrentMax:	DW 0
WFCurrentMin: 	DW 0
WFCurrentValue: DW 0
WFInvalid:      DW 32767
WFABunchOfOnes: DW &B1111111111111111
WFArrayIndex:   DW 0
WFNumOfWalls:   DW 0
FoundWalls:     DW 720					;start of array of found walls
wall2:          DW 0
wall3:          DW 0
wall4:          DW 0
wall5:          DW 0
wall6:          DW 0
wall7:          DW 0
wall8:          DW 0
wall9:          DW 0
wall10:         DW 0
wall11:         DW 0
wall12:         DW 0
wall13:         DW 0
wall14:         DW 0
wall15:         DW 0
wall16:         DW 0
wall17:         DW 0
wall18:         DW 0

findWallsDA:  LOAD DACurrentWall
				ADDI 1
				STORE DACurrentWall
				SUB WFNumOfWalls
				JZERO Main		;to change Didn't find anything
				CALL Wait4User
				LOADI FoundWalls
				ADD DACurrentWall
				STORE ArrayIndex        ;array index refers to the current wall
				ILOAD ArrayIndex
				STORE DACurrValue  
				ADDI 80
				STORE DAMin
				ADDI 20
				STORE DAMax				;max and min are set for this iteration
				LOADI 0
				STORE DATestingWall
				
DAInner:      LOADI FoundWalls
				ADD DATestingWall
				STORE ArrayIndex
				ILOAD ArrayIndex
				STORE DATestValue
				SUB DAMin         	;compare with minimum allowable value
				JNEG DAFail1
				LOAD DATestValue     
				SUB DAMax             ;compare with maximum allowable value
				JPOS DAFail1
				JUMP DASuccess
				
DAFail1:      LOAD DAMax
				ADDI -360
				STORE DAMax
				LOAD DAMin
				ADDI -360
				STORE DAMin
				LOAD DATestValue
				SUB DAMin         	;compare with minimum allowable value
				JPOS DAFail2
				LOAD DATestValue     
				SUB DAMax             ;compare with maximum allowable value
				JNEG DAFail2
				JUMP DASuccess
				
DAFail2:      LOAD DATestingWall
				ADDI 1
				STORE DATestingWall	;increment testing wall
				SUB WFNumOfWalls
				JZERO findWallsDA
				JUMP DAInner            ;keep loopin'
				
DASuccess:	LOADI 2
				STORE W4ULEDS
				LOAD DACurrValue
				STORE smallWall
				CALL Wait4User
				LOAD DATestValue
				STORE bigWall
				CALL Wait4User
				JUMP GoIntoCenterLane				;we found two walls that are 90 degrees apart


DACurrentWall: DW -1
DACurrValue:	DW 0
DATestingWall: DW 0
DATestValue:	DW 0
DAMin:		DW 85
DAMax:		DW 95
;bigWall:        DW 0
;smallWall:      DW 0

	
	;***************************************************************
;* Joshua Davis Subroutines
;***************************************************************
GoIntoCenterLane:
	LOADI 3
	STORE W4ULEDS
	;Turn towards the Large Wall
	LOAD	BigWall  
	STORE 	DTheta
	LOAD    Mask2   ;Enable Sensor 2 for measurements
	OUT     SONAREN
	
	IN      DIST2
	;CALL    Wait4User
	LOADI   DataArray
	ADD     SmallWall
	STORE   ArrayIndex
	ILOAD   ArrayIndex
	STORE   SmallWallDist
	CALL    Wait4User
	LOADI   DataArray
	ADD     BigWall
	STORE   ArrayIndex
	ILOAD   ArrayIndex
	STORE   BigWallDist
	CALL    Wait4User
	SUB	    DistBWall2SafeM        ;Detect how far away bot is from wall  
	JNEG	MoveAwayFromBigWall    ;If Dist2 < SafeDistance move away from wall
	JUMP	MoveToBigWall          ;If Dist2 > SafeDistance move toward wall
	
	
MoveAwayFromBigWall: ; Keep Moving away from large Wall until 
	LOADI 4
	STORE W4ULEDS
	LOAD	RMid
	STORE	DVel

CheckSafeNeg:	
	IN      DIST2
	SUB	    DistBWall2SafeM
	JNEG    CheckSafeNeg      ; If Distance < SafeDistance keep moving back 
	CALL    Wait4User
	JUMP	GoToHomeFromLane

MoveToBigWall: ; Keep Moving towards Large Wall until 
	LOADI 4
	STORE W4ULEDS
	LOAD	FMid
	STORE	DVel

CheckSafePos:	
	IN      DIST2
	SUB	    DistBWall2SafeM
	JPOS    CheckSafePos     ; If Distance > SafeDistance keep moving forward
	;CALL    Wait4User
	JUMP	GoToHomeFromLane
	
GoToHomeFromLane:
	LOADI	0
	STORE	DVel
	CALL	Wait4User
	LOADI	270
	STORE	DTheta
	LOAD 	RMid 
	STORE 	DVel
	LOAD    Mask2 
	OUT     SONAREN
	
MoveForwardUntilDest:
	IN		DIST2
	
	SUB		DistSWall2Home
	JNEG    MoveForwardUntilDest
	LOAD	Zero
	STORE 	DVel
	JUMP 	Die
	
	
DistBWall2SafeM: DW 1922  ; 200cm Distance from large wall to middle of safe lane 
DistSWall2Home:  DW 7688  ; 800 cm Distance from small wall to middle of home area 
BigWall:    DW 0          ; Angle of the big wall relative to the robot 
BigWallDist: DW 0
SmallWall:  DW 0          ; Angle of the small wall relative to the robot
SmallWallDist: DW 0
Dist2Travel: DW 0 
	
; FindClosest subroutine will go through the acquired data
; and return the angle of the closest sonar reading.
FindClosest:
	LOADI  DataArray   ; get the array start address
	STORE  ArrayIndex
	STORE  CloseIndex  ; keep track of shortest distance
	ADDI   360
	STORE  EndIndex
	ILOAD  ArrayIndex  ; get the first entry of array
	STORE  CloseVal    ; keep track of shortest distance
FCLoop:
	LOAD   ArrayIndex
	ADDI   1
	STORE  ArrayIndex  ; move to next entry
	XOR    EndIndex    ; compare with end index
	JZERO  FCDone
	ILOAD  ArrayIndex  ; get the data
	SUB    CloseVal    ; compare with current min
	JPOS   FCLoop      ; not closer; move on
	ILOAD  ArrayIndex  ; new minimum
	STORE  CloseVal
	LOAD   ArrayIndex
	STORE  CloseIndex
	JUMP   FCLoop
FCDone:
	; Need to convert the index to an angle.
	; Since the data is stored according to angle,
	; that means we just need the value position.
	LOADI  DataArray   ; start address
	SUB    CloseIndex  ; start address - entry address
	CALL   Neg         ; entry address - start address
	RETURN
	
	EndIndex:   DW 0
	CloseIndex: DW 0
	CloseVal:   DW 0



Die:
; Sometimes it's useful to permanently stop execution.
; This will also catch the execution if it accidentally
; falls through from above.
	CLI    &B1111      ; disable all interrupts
	LOAD   Zero        ; Stop everything.
	OUT    LVELCMD
	OUT    RVELCMD
	OUT    SONAREN
	LOAD   DEAD        ; An indication that we are dead
	OUT    SSEG2       ; "dEAd" on the sseg
Forever:
	JUMP   Forever     ; Do this forever.
	DEAD:  DW &HDEAD   ; Example of a "local" variable


; Timer ISR.  Currently just calls the movement control code.
; You could, however, do additional tasks here if desired.
CTimer_ISR:
	CALL   ControlMovement
	RETI   ; return from ISR
	
	
; Control code.  If called repeatedly, this code will attempt
; to control the robot to face the angle specified in DTheta
; and match the speed specified in DVel
DTheta:    DW 0
DVel:      DW 0
ControlMovement:
	LOADI  50          ; used for the CapValue subroutine
	STORE  MaxVal
	CALL   GetThetaErr ; get the heading error
	; A simple way to get a decent velocity value
	; for turning is to multiply the angular error by 4
	; and add ~50.
	SHIFT  2
	STORE  CMAErr      ; hold temporarily
	SHIFT  2           ; multiply by another 4
	CALL   CapValue    ; get a +/- max of 50
	ADD    CMAErr
	STORE  CMAErr      ; now contains a desired differential

	
	; For this basic control method, simply take the
	; desired forward velocity and add the differential
	; velocity for each wheel when turning is needed.
	LOADI  510
	STORE  MaxVal
	LOAD   DVel
	CALL   CapValue    ; ensure velocity is valid
	STORE  DVel        ; overwrite any invalid input
	ADD    CMAErr
	CALL   CapValue    ; ensure velocity is valid
	STORE  CMAR
	LOAD   CMAErr
	CALL   Neg         ; left wheel gets negative differential
	ADD    DVel
	CALL   CapValue
	STORE  CMAL

	; ensure enough differential is applied
	LOAD   CMAErr
	SHIFT  1           ; double the differential
	STORE  CMAErr
	LOAD   CMAR
	SUB    CMAL        ; calculate the actual differential
	SUB    CMAErr      ; should be 0 if nothing got capped
	JZERO  CMADone
	; re-apply any missing differential
	STORE  CMAErr      ; the missing part
	ADD    CMAL
	CALL   CapValue
	STORE  CMAL
	LOAD   CMAR
	SUB    CMAErr
	CALL   CapValue
	STORE  CMAR

CMADone:
	LOAD   CMAL
	OUT    LVELCMD
	LOAD   CMAR
	OUT    RVELCMD

	RETURN
	CMAErr: DW 0       ; holds angle error velocity
	CMAL:    DW 0      ; holds temp left velocity
	CMAR:    DW 0      ; holds temp right velocity

; Returns the current angular error wrapped to +/-180
GetThetaErr:
	; convenient way to get angle error in +/-180 range is
	; ((error + 180) % 360 ) - 180
	IN     THETA
	SUB    DTheta      ; actual - desired angle
	CALL   Neg         ; desired - actual angle
	ADDI   180
	CALL   Mod360
	ADDI   -180
	RETURN

; caps a value to +/-MaxVal
CapValue:
	SUB     MaxVal
	JPOS    CapVelHigh
	ADD     MaxVal
	ADD     MaxVal
	JNEG    CapVelLow
	SUB     MaxVal
	RETURN
CapVelHigh:
	LOAD    MaxVal
	RETURN
CapVelLow:
	LOAD    MaxVal
	CALL    Neg
	RETURN
	MaxVal: DW 510

;***************************************************************
;* Useful Subroutines
;***************************************************************
SendData:
	; Get the memory address of the array and store it
	LOADI   DataArray
	STORE   ArrayIndex
	ADDI    360
	STORE   Temp        ; Also store the end address
SDLoop1:
	IN      UART_RDY    ; get the UART status
	SHIFT   -9          ; check if the write buffer is full
	JPOS    SDLoop1
	
	ILOAD   ArrayIndex
	SHIFT   -8          ; move high byte to low byte
	OUT     UART_DAT
	
SDLoop2:
	IN      UART_RDY    ; get the UART status
	SHIFT   -9          ; check if the write buffer is full
	JPOS    SDLoop2
	
	ILOAD   ArrayIndex
	OUT     UART_DAT    ; send low byte
	
	LOAD    ArrayIndex
	ADDI    1           ; increment index
	STORE   ArrayIndex
	SUB     Temp        ; check if at end of array
	JNEG    SDLoop1
	JUMP    Die         ; when done, go to infinite loop


;*******************************************************************************
; Mod360: modulo 360
; Returns AC%360 in AC
; Written by Kevin Johnson.  No licence or copyright applied.
;*******************************************************************************
Mod360:
	; easy modulo: subtract 360 until negative then add 360 until not negative
	JNEG   M360N
	ADDI   -360
	JUMP   Mod360
M360N:
	ADDI   360
	JNEG   M360N
	RETURN

;*******************************************************************************
; Abs: 2's complement absolute value
; Returns abs(AC) in AC
; Neg: 2's complement negation
; Returns -AC in AC
; Written by Kevin Johnson.  No licence or copyright applied.
;*******************************************************************************
Abs:
	JPOS   Abs_r
Neg:
	XOR    NegOne       ; Flip all bits
	ADDI   1            ; Add one (i.e. negate number)
Abs_r:
	RETURN

;******************************************************************************;
; Atan2: 4-quadrant arctangent calculation                                     ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; Original code by Team AKKA, Spring 2015.                                     ;
; Based on methods by Richard Lyons                                            ;
; Code updated by Kevin Johnson to use software mult and div                   ;
; No license or copyright applied.                                             ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; To use: store dX and dY in global variables AtanX and AtanY.                 ;
; Call Atan2                                                                   ;
; Result (angle [0,359]) is returned in AC                                     ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; Requires additional subroutines:                                             ;
; - Mult16s: 16x16->32bit signed multiplication                                ;
; - Div16s: 16/16->16R16 signed division                                       ;
; - Abs: Absolute value                                                        ;
; Requires additional constants:                                               ;
; - One:     DW 1                                                              ;
; - NegOne:  DW 0                                                              ;
; - LowByte: DW &HFF                                                           ;
;******************************************************************************;
Atan2:
	LOAD   AtanY
	CALL   Abs          ; abs(y)
	STORE  AtanT
	LOAD   AtanX        ; abs(x)
	CALL   Abs
	SUB    AtanT        ; abs(x) - abs(y)
	JNEG   A2_sw        ; if abs(y) > abs(x), switch arguments.
	LOAD   AtanX        ; Octants 1, 4, 5, 8
	JNEG   A2_R3
	CALL   A2_calc      ; Octants 1, 8
	JNEG   A2_R1n
	RETURN              ; Return raw value if in octant 1
A2_R1n: ; region 1 negative
	ADDI   360          ; Add 360 if we are in octant 8
	RETURN
A2_R3: ; region 3
	CALL   A2_calc      ; Octants 4, 5            
	ADDI   180          ; theta' = theta + 180
	RETURN
A2_sw: ; switch arguments; octants 2, 3, 6, 7 
	LOAD   AtanY        ; Swap input arguments
	STORE  AtanT
	LOAD   AtanX
	STORE  AtanY
	LOAD   AtanT
	STORE  AtanX
	JPOS   A2_R2        ; If Y positive, octants 2,3
	CALL   A2_calc      ; else octants 6, 7
	CALL   Neg          ; Negatge the number
	ADDI   270          ; theta' = 270 - theta
	RETURN
A2_R2: ; region 2
	CALL   A2_calc      ; Octants 2, 3
	CALL   Neg          ; negate the angle
	ADDI   90           ; theta' = 90 - theta
	RETURN
A2_calc:
	; calculates R/(1 + 0.28125*R^2)
	LOAD   AtanY
	STORE  d16sN        ; Y in numerator
	LOAD   AtanX
	STORE  d16sD        ; X in denominator
	CALL   A2_div       ; divide
	LOAD   dres16sQ     ; get the quotient (remainder ignored)
	STORE  AtanRatio
	STORE  m16sA
	STORE  m16sB
	CALL   A2_mult      ; X^2
	STORE  m16sA
	LOAD   A2c
	STORE  m16sB
	CALL   A2_mult
	ADDI   256          ; 256/256+0.28125X^2
	STORE  d16sD
	LOAD   AtanRatio
	STORE  d16sN        ; Ratio in numerator
	CALL   A2_div       ; divide
	LOAD   dres16sQ     ; get the quotient (remainder ignored)
	STORE  m16sA        ; <= result in radians
	LOAD   A2cd         ; degree conversion factor
	STORE  m16sB
	CALL   A2_mult      ; convert to degrees
	STORE  AtanT
	SHIFT  -7           ; check 7th bit
	AND    One
	JZERO  A2_rdwn      ; round down
	LOAD   AtanT
	SHIFT  -8
	ADDI   1            ; round up
	RETURN
A2_rdwn:
	LOAD   AtanT
	SHIFT  -8           ; round down
	RETURN
A2_mult: ; multiply, and return bits 23..8 of result
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8            ; move high word of result up 8 bits
	STORE  mres16sH
	LOAD   mres16sL
	SHIFT  -8           ; move low word of result down 8 bits
	AND    LowByte
	OR     mres16sH     ; combine high and low words of result
	RETURN
A2_div: ; 16-bit division scaled by 256, minimizing error
	LOADI  9            ; loop 8 times (256 = 2^8)
	STORE  AtanT
A2_DL:
	LOAD   AtanT
	ADDI   -1
	JPOS   A2_DN        ; not done; continue shifting
	CALL   Div16s       ; do the standard division
	RETURN
A2_DN:
	STORE  AtanT
	LOAD   d16sN        ; start by trying to scale the numerator
	SHIFT  1
	XOR    d16sN        ; if the sign changed,
	JNEG   A2_DD        ; switch to scaling the denominator
	XOR    d16sN        ; get back shifted version
	STORE  d16sN
	JUMP   A2_DL
A2_DD:
	LOAD   d16sD
	SHIFT  -1           ; have to scale denominator
	STORE  d16sD
	JUMP   A2_DL
AtanX:      DW 0
AtanY:      DW 0
AtanRatio:  DW 0        ; =y/x
AtanT:      DW 0        ; temporary value
A2c:        DW 72       ; 72/256=0.28125, with 8 fractional bits
A2cd:       DW 14668    ; = 180/pi with 8 fractional bits

;*******************************************************************************
; Mult16s:  16x16 -> 32-bit signed multiplication
; Based on Booth's algorithm.
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: does not work with factor B = -32768 (most-negative number).
; To use:
; - Store factors in m16sA and m16sB.
; - Call Mult16s
; - Result is stored in mres16sH and mres16sL (high and low words).
;*******************************************************************************
Mult16s:
	LOADI  0
	STORE  m16sc        ; clear carry
	STORE  mres16sH     ; clear result
	LOADI  16           ; load 16 to counter
Mult16s_loop:
	STORE  mcnt16s      
	LOAD   m16sc        ; check the carry (from previous iteration)
	JZERO  Mult16s_noc  ; if no carry, move on
	LOAD   mres16sH     ; if a carry, 
	ADD    m16sA        ;  add multiplicand to result H
	STORE  mres16sH
Mult16s_noc: ; no carry
	LOAD   m16sB
	AND    One          ; check bit 0 of multiplier
	STORE  m16sc        ; save as next carry
	JZERO  Mult16s_sh   ; if no carry, move on to shift
	LOAD   mres16sH     ; if bit 0 set,
	SUB    m16sA        ;  subtract multiplicand from result H
	STORE  mres16sH
Mult16s_sh:
	LOAD   m16sB
	SHIFT  -1           ; shift result L >>1
	AND    c7FFF        ; clear msb
	STORE  m16sB
	LOAD   mres16sH     ; load result H
	SHIFT  15           ; move lsb to msb
	OR     m16sB
	STORE  m16sB        ; result L now includes carry out from H
	LOAD   mres16sH
	SHIFT  -1
	STORE  mres16sH     ; shift result H >>1
	LOAD   mcnt16s
	ADDI   -1           ; check counter
	JPOS   Mult16s_loop ; need to iterate 16 times
	LOAD   m16sB
	STORE  mres16sL     ; multiplier and result L shared a word
	RETURN              ; Done
c7FFF: DW &H7FFF
m16sA: DW 0 ; multiplicand
m16sB: DW 0 ; multipler
m16sc: DW 0 ; carry
mcnt16s: DW 0 ; counter
mres16sL: DW 0 ; result low
mres16sH: DW 0 ; result high

;*******************************************************************************
; Div16s:  16/16 -> 16 R16 signed division
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: results undefined if denominator = 0.
; To use:
; - Store numerator in d16sN and denominator in d16sD.
; - Call Div16s
; - Result is stored in dres16sQ and dres16sR (quotient and remainder).
; Requires Abs subroutine
;*******************************************************************************
Div16s:
	LOADI  0
	STORE  dres16sR     ; clear remainder result
	STORE  d16sC1       ; clear carry
	LOAD   d16sN
	XOR    d16sD
	STORE  d16sS        ; sign determination = N XOR D
	LOADI  17
	STORE  d16sT        ; preload counter with 17 (16+1)
	LOAD   d16sD
	CALL   Abs          ; take absolute value of denominator
	STORE  d16sD
	LOAD   d16sN
	CALL   Abs          ; take absolute value of numerator
	STORE  d16sN
Div16s_loop:
	LOAD   d16sN
	SHIFT  -15          ; get msb
	AND    One          ; only msb (because shift is arithmetic)
	STORE  d16sC2       ; store as carry
	LOAD   d16sN
	SHIFT  1            ; shift <<1
	OR     d16sC1       ; with carry
	STORE  d16sN
	LOAD   d16sT
	ADDI   -1           ; decrement counter
	JZERO  Div16s_sign  ; if finished looping, finalize result
	STORE  d16sT
	LOAD   dres16sR
	SHIFT  1            ; shift remainder
	OR     d16sC2       ; with carry from other shift
	SUB    d16sD        ; subtract denominator from remainder
	JNEG   Div16s_add   ; if negative, need to add it back
	STORE  dres16sR
	LOADI  1
	STORE  d16sC1       ; set carry
	JUMP   Div16s_loop
Div16s_add:
	ADD    d16sD        ; add denominator back in
	STORE  dres16sR
	LOADI  0
	STORE  d16sC1       ; clear carry
	JUMP   Div16s_loop
Div16s_sign:
	LOAD   d16sN
	STORE  dres16sQ     ; numerator was used to hold quotient result
	LOAD   d16sS        ; check the sign indicator
	JNEG   Div16s_neg
	RETURN
Div16s_neg:
	LOAD   dres16sQ     ; need to negate the result
	CALL   Neg
	STORE  dres16sQ
	RETURN	
d16sN: DW 0 ; numerator
d16sD: DW 0 ; denominator
d16sS: DW 0 ; sign value
d16sT: DW 0 ; temp counter
d16sC1: DW 0 ; carry value
d16sC2: DW 0 ; carry value
dres16sQ: DW 0 ; quotient result
dres16sR: DW 0 ; remainder result

;*******************************************************************************
; L2Estimate:  Pythagorean distance estimation
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: this is *not* an exact function.  I think it's most wrong
; on the axes, and maybe at 45 degrees.
; To use:
; - Store X and Y offset in L2X and L2Y.
; - Call L2Estimate
; - Result is returned in AC.
; Result will be in same units as inputs.
; Requires Abs and Mult16s subroutines.
;*******************************************************************************
L2Estimate:
	; take abs() of each value, and find the largest one
	LOAD   L2X
	CALL   Abs
	STORE  L2T1
	LOAD   L2Y
	CALL   Abs
	SUB    L2T1
	JNEG   GDSwap    ; swap if needed to get largest value in X
	ADD    L2T1
CalcDist:
	; Calculation is max(X,Y)*0.961+min(X,Y)*0.406
	STORE  m16sa
	LOADI  246       ; max * 246
	STORE  m16sB
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8
	STORE  L2T2
	LOAD   mres16sL
	SHIFT  -8        ; / 256
	AND    LowByte
	OR     L2T2
	STORE  L2T3
	LOAD   L2T1
	STORE  m16sa
	LOADI  104       ; min * 104
	STORE  m16sB
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8
	STORE  L2T2
	LOAD   mres16sL
	SHIFT  -8        ; / 256
	AND    LowByte
	OR     L2T2
	ADD    L2T3     ; sum
	RETURN
GDSwap: ; swaps the incoming X and Y
	ADD    L2T1
	STORE  L2T2
	LOAD   L2T1
	STORE  L2T3
	LOAD   L2T2
	STORE  L2T1
	LOAD   L2T3
	JUMP   CalcDist
L2X:  DW 0
L2Y:  DW 0
L2T1: DW 0
L2T2: DW 0
L2T3: DW 0


; Subroutine to wait (block) for 1 second
Wait1:
	OUT    TIMER
Wloop:
	IN     TIMER
	OUT    XLEDS       ; User-feedback that a pause is occurring.
	ADDI   -10         ; 1 second at 10Hz.
	JNEG   Wloop
	RETURN

; This subroutine will get the battery voltage,
; and stop program execution if it is too low.
; SetupI2C must be executed prior to this.
BattCheck:
	CALL   GetBattLvl
	JZERO  BattCheck   ; A/D hasn't had time to initialize
	SUB    MinBatt
	JNEG   DeadBatt
	ADD    MinBatt     ; get original value back
	RETURN
; If the battery is too low, we want to make
; sure that the user realizes it...
DeadBatt:
	LOADI  &H20
	OUT    BEEP        ; start beep sound
	CALL   GetBattLvl  ; get the battery level
	OUT    SSEG1       ; display it everywhere
	OUT    SSEG2
	OUT    LCD
	LOAD   Zero
	ADDI   -1          ; 0xFFFF
	OUT    LEDS        ; all LEDs on
	OUT    XLEDS
	CALL   Wait1       ; 1 second
	LOADI  &H140       ; short, high-pitched beep
	OUT    BEEP        ; stop beeping
	LOAD   Zero
	OUT    LEDS        ; LEDs off
	OUT    XLEDS
	CALL   Wait1       ; 1 second
	JUMP   DeadBatt    ; repeat forever
	
; Subroutine to read the A/D (battery voltage)
; Assumes that SetupI2C has been run
GetBattLvl:
	LOAD   I2CRCmd     ; 0x0190 (write 0B, read 1B, addr 0x90)
	OUT    I2C_CMD     ; to I2C_CMD
	OUT    I2C_RDY     ; start the communication
	CALL   BlockI2C    ; wait for it to finish
	IN     I2C_DATA    ; get the returned data
	RETURN

; Subroutine to configure the I2C for reading batt voltage
; Only needs to be done once after each reset.
SetupI2C:
	CALL   BlockI2C    ; wait for idle
	LOAD   I2CWCmd     ; 0x1190 (write 1B, read 1B, addr 0x90)
	OUT    I2C_CMD     ; to I2C_CMD register
	LOAD   Zero        ; 0x0000 (A/D port 0, no increment)
	OUT    I2C_DATA    ; to I2C_DATA register
	OUT    I2C_RDY     ; start the communication
	CALL   BlockI2C    ; wait for it to finish
	RETURN
	
; Subroutine to block until I2C device is idle
BlockI2C:
	LOAD   Zero
	STORE  Temp        ; Used to check for timeout
BI2CL:
	LOAD   Temp
	ADDI   1           ; this will result in ~0.1s timeout
	STORE  Temp
	JZERO  I2CError    ; Timeout occurred; error
	IN     I2C_RDY     ; Read busy signal
	JPOS   BI2CL       ; If not 0, try again
	RETURN             ; Else return
I2CError:
	LOAD   Zero
	ADDI   &H12C       ; "I2C"
	OUT    SSEG1
	OUT    SSEG2       ; display error message
	JUMP   I2CError

;***************************************************************
;* Variables
;***************************************************************
Temp:     DW 0 ; "Temp" is not a great name, but can be useful

;***************************************************************
;* Constants
;* (though there is nothing stopping you from writing to these)
;***************************************************************
NegOne:   DW -1
Zero:     DW 0
One:      DW 1
Two:      DW 2
Three:    DW 3
Four:     DW 4
Five:     DW 5
Six:      DW 6
Seven:    DW 7
Eight:    DW 8
Nine:     DW 9
Ten:      DW 10

; Some bit masks.
; Masks of multiple bits can be constructed by ORing these
; 1-bit masks together.
Mask0:    DW &B00000001
Mask1:    DW &B00000010
Mask2:    DW &B00000100
Mask3:    DW &B00001000
Mask4:    DW &B00010000
Mask5:    DW &B00100000
Mask6:    DW &B01000000
Mask7:    DW &B10000000
LowByte:  DW &HFF      ; binary 00000000 1111111
LowNibl:  DW &HF       ; 0000 0000 0000 1111

; some useful movement values
OneMeter: DW 961       ; ~1m in 1.04mm units
HalfMeter: DW 481      ; ~0.5m in 1.04mm units
Ft2:      DW 586       ; ~2ft in 1.04mm units
Ft3:      DW 879
Ft4:      DW 1172
Deg90:    DW 90        ; 90 degrees in odometer units
Deg180:   DW 180       ; 180
Deg270:   DW 270       ; 270
Deg360:   DW 360       ; can never actually happen; for math only
FSlow:    DW 100       ; 100 is about the lowest velocity value that will move
RSlow:    DW -100
FMid:     DW 350       ; 350 is a medium speed
RMid:     DW -350
FFast:    DW 500       ; 500 is almost max speed (511 is max)
RFast:    DW -500

MinBatt:  DW 140       ; 14.0V - minimum safe battery voltage
I2CWCmd:  DW &H1190    ; write one i2c byte, read one byte, addr 0x90
I2CRCmd:  DW &H0190    ; write nothing, read one byte, addr 0x90

DataArray:
	DW 0
;***************************************************************
;* IO address space map
;***************************************************************
SWITCHES: EQU &H00  ; slide switches
LEDS:     EQU &H01  ; red LEDs
TIMER:    EQU &H02  ; timer, usually running at 10 Hz
XIO:      EQU &H03  ; pushbuttons and some misc. inputs
SSEG1:    EQU &H04  ; seven-segment display (4-digits only)
SSEG2:    EQU &H05  ; seven-segment display (4-digits only)
LCD:      EQU &H06  ; primitive 4-digit LCD display
XLEDS:    EQU &H07  ; Green LEDs (and Red LED16+17)
BEEP:     EQU &H0A  ; Control the beep
CTIMER:   EQU &H0C  ; Configurable timer for interrupts
LPOS:     EQU &H80  ; left wheel encoder position (read only)
LVEL:     EQU &H82  ; current left wheel velocity (read only)
LVELCMD:  EQU &H83  ; left wheel velocity command (write only)
RPOS:     EQU &H88  ; same values for right wheel...
RVEL:     EQU &H8A  ; ...
RVELCMD:  EQU &H8B  ; ...
I2C_CMD:  EQU &H90  ; I2C module's CMD register,
I2C_DATA: EQU &H91  ; ... DATA register,
I2C_RDY:  EQU &H92  ; ... and BUSY register
UART_DAT: EQU &H98  ; UART data
UART_RDY: EQU &H99  ; UART status
SONAR:    EQU &HA0  ; base address for more than 16 registers....
DIST0:    EQU &HA8  ; the eight sonar distance readings
DIST1:    EQU &HA9  ; ...
DIST2:    EQU &HAA  ; ...
DIST3:    EQU &HAB  ; ...
DIST4:    EQU &HAC  ; ...
DIST5:    EQU &HAD  ; ...
DIST6:    EQU &HAE  ; ...
DIST7:    EQU &HAF  ; ...
SONALARM: EQU &HB0  ; Write alarm distance; read alarm register
SONARINT: EQU &HB1  ; Write mask for sonar interrupts
SONAREN:  EQU &HB2  ; register to control which sonars are enabled
XPOS:     EQU &HC0  ; Current X-position (read only)
YPOS:     EQU &HC1  ; Y-position
THETA:    EQU &HC2  ; Current rotational position of robot (0-359)
RESETPOS: EQU &HC3  ; write anything here to reset odometry to 0
RIN:      EQU &HC8
LIN:      EQU &HC9
IR_HI:    EQU &HD0  ; read the high word of the IR receiver (OUT will clear both words)
IR_LO:    EQU &HD1  ; read the low word of the IR receiver (OUT will clear both words)