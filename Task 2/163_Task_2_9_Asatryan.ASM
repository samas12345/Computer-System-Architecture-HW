
; HSE 163, Num. 9: Samvel Asatryan
; This is an app for copying odd numbers of inputed array into new array and printing them both

format  PE console ; app's format
include "C:\Users\hayka\OneDrive\Desktop\fasmw17325\INCLUDE\WIN32AX.INC" ; WIN32AX.INC location

entry main ; Entry point of programm

section '.data' import data readable writable; Section for storing data

        library kernel, 'kernel32.dll',\
                msvcrt, 'msvcrt.dll',\
                user32, 'USER32.dll'
        import kernel,\
              ExitProcess, 'ExitProcess'
        import msvcrt,\
               printf, 'printf',\
               scanf, 'scanf',\
               getch, '_getch'

        endl FIX 10, 13 ; New line character


        arraySize dd ? ;input array size
        arrayBSize dd ? ;out array size
        element dd ?
        tmp dd ?
        ; User fields for output

        arrayA dd 10000 dup(?) ;input array
        arrayB dd 10000 dup(?) ;out array

        ask_size DB "Please enter the size of array: ", 0
        ask_element DB "No[%d]: ", 0
        print_element DB "%d  ", 0

        printingA DB "Inputed array: ", 0
        printingB DB endl, "Odd numbers of inputed array: ", 0

        errorMsg DB "Incorrect input. Please restart program.", 0

        formatin db '%d', 0

section '.text' code readable executable ; Section of code

; Main
proc main
        ;asks size of array
        push ask_size
        call [printf]
        add esp, 4

        ;saves array size in arraySize
        push arraySize
        push formatin
        call [scanf]
        mov esi, [arraySize]

        cmp esi, 0
        je error
        add esp, 8

        mov ebx, arrayA

        mov esi, 0

        ;read array
        readLoop:
                cmp esi, [arraySize]  ;iterator of loop
                jge endOfInput

                push esi

                ;asks element
                push ask_element
                call [printf]
                add esp, 4

                pop esi
                push esi

                ;saves inputed element
                push element
                push formatin
                call [scanf]
                mov ecx, [element]
                add esp, 8

                cmp ecx, 0
                je error

                pop esi
                push esi

                ;saves inputed element in arrayA
                mov edx, [element]
                mov [ebx], edx
                add ebx, 4

                pop esi
                inc esi


                jmp readLoop


        endOfInput:

        mov edx, arrayA
        mov esi, 0
        mov eax, 0
        mov ecx, 0

        ;calls proc MakeB to copy odd elements of inputed array
        call MakeB
        mov [arrayBSize], eax

        ;prints inputed array
        push printingA
        call [printf]
        add esp, 4

        mov esi, 0
        mov eax, 0

        writeALoop:
                cmp esi, [arraySize] ;loop iterator
                jge endOfOutput

                push esi
                push eax

                ;prints next element of array
                push dword[arrayA + eax]
                push print_element
                call [printf]
                add esp, 8

                pop eax
                pop esi

                inc esi
                add eax, 4
                jmp writeALoop

        endOfOutput:

        ;prints resulting array
        push printingB
        call [printf]
        add esp, 4

        mov esi, 0
        mov eax, 0


        writeBLoop:
                cmp esi, [arrayBSize] ;iterator of loop
                jge endOfBOutput

                push esi
                push eax

                ;printing the next element of array
                push dword[arrayB + eax]
                push print_element
                call [printf]
                add esp, 8

                pop eax
                pop esi

                inc esi
                add eax, 4
                jmp writeBLoop

        endOfBOutput:



        call [getch]
        push 0
        call [ExitProcess]

        error:
                push errorMsg
                call [printf]
                add esp, 4
                call [getch]
                push 0
                call [ExitProcess]

endp

;process MakeB which copies odd elements in new array arrayB. Takes argument arrayA which is stored in edx
proc MakeB
     enter 0, 0
     checkLoop:
        cmp esi, [arraySize]   ;iterator of loop
        jge endOfCheck

        mov ebx, [edx + 4*esi]  ;next element of arrayA

        inc esi

        ;storing element in ecx
        mov [tmp], ebx
        mov ecx, [tmp]

        and cx, 1; if the smallest bit is 1 then number is odd
        jnz addToB

        jmp checkLoop

     addToB: ;if number is odd, storing it in arrayB
        mov dword[arrayB + eax*4], ebx
        inc eax
        jmp checkLoop

     endOfCheck:
        leave
        ret                                                                                                    4
endp


