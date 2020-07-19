import sys

lines = open("kbd_hw.txt").readlines()

translate = {"Help": "KPPLUS",
             "Mark": "F11", 
             "Bound": "F12",
             "Finish": "KPMULT",
             "Jump": "",  # XXX
             "Move": "",  # XXX
             "Copy": "",  # XXX
             "Col": "",  # XXX
             "Para": "",  # XXX
             "Sent": "",  # XXX
             "Word": "",  # XXX
             "Line": "",  # XXX
             "System": "",  # XXX
             "Alt": "",  # XXX
             "00": "",  # XXX
             "Delete": "",  # XXX
             "Next": "",  # XXX
             "Go": "KPMINUS", 
             "Cursor Up)": "UARR",
             "Page Prev": "PGUP",
             "Page Next": "PGDN",
             "Cancel": "ESC",
             "Back Space": "BS",
             "Tab": "TAB",
             "Return": "ENTER",
             "(Cursor Dn)": "DARR",
             "(Cursor Left)": "LARR",
             "(Cursor Right)": "RARR",
             "Scroll Next": "HOME",
             "Scroll Prev": "END",
             "(Space Bar)": "SPACE",
             "Delete Char": "DEL",
             "X (Multiply)": "",  # XXX
             "(Divide)": "",  # XXX
             "Action": "LALT",
             "Over Type": "INS",
             "Lock": "CAPSLOCK",
             "Shift (Left)": "LSHIFT",
             "Shift (Right)": "RSHIFT",
             "Code (Left)": "LCTRL",
             "Code (Right)": "RCTRL"
}

print "from ps2_scancode_list import *"
print "convergent_codes=["

for l in lines:
    l = l.strip()
    parts = l.split("|")
    if len(parts) < 4:
        #print >> sys.stderr, "# ignored line: %s" % l
        continue

    code = parts[0].strip()
    hexref = parts[1].strip()
    legend = parts[2].strip()
    keynumber = parts[3].strip().split(" ")[0].strip()

    legend = translate.get(legend, legend)

    if keynumber == "92&92A":
        keynumber = "92"

    if legend == "'":
        legend = '"\'"'

    if len(legend)==1:
        legend = "'" + legend + "'"

    if (not code) or (not hexref) or (not legend) or (not keynumber):
        #print >> sys.stderr, "# ignored line: %s" % l
        continue

    print "(%4s, %10s, 0x%s)," % (keynumber, legend, code)

print "]"