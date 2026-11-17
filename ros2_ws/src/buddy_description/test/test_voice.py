#!/usr/bin/env python3
"""
Intent matching and dialogue logic. No audio, no network, no ROS.

These cover the parts where a bug is expensive rather than annoying: a missed
"stop" command, and a wrong name written into the face database.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'scripts'))

from buddy_intents import (CHAT, COME, FOLLOW, NAME, NO, STOP, WANDER, YES,
                           classify, extract_name, normalise)
from buddy_dialogue import (ASK_NAME, CHATTING, CONFIRM_NAME, Dialogue, ENROLL,
                            IDLE, LISTEN, SAY, SET_MODE)

fails = []


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}{'  ' + detail if detail else ''}")
    if not cond:
        fails.append(name)


def kinds(acts):
    return [a.kind for a in acts]


def value(acts, kind):
    for a in acts:
        if a.kind == kind:
            return a.value
    return None


print('1. Recogniser output is UPPERCASE with no punctuation.')
check('normalise handles raw STT', normalise("MY NAME IS ALEX.") == 'my name is alex')
check('uppercase command matches', classify('STOP').kind == STOP)

print('\n2. STOP is matched loosely and always wins.')
for phrase in ['stop', 'STOP', 'stop it', 'please stop', 'wait', 'halt',
               'hold on', 'stop there']:
    check(f'stop: {phrase!r}', classify(phrase).kind == STOP)
# A false positive stops a slow robot (harmless); a false negative does not stop
# it when asked (not harmless). Loose matching is the right trade.
check('stop beats a name in the same breath',
      classify('stop my name is alex').kind == STOP)

print('\n3. Names are extracted, and non-names are not.')
for phrase, want in [('my name is alex', 'Alex'), ('i am sam', 'Sam'),
                     ('call me terry', 'Terry'), ("i'm jordan", 'Jordan'),
                     ('my name is mary jane', 'Mary Jane')]:
    got = extract_name(phrase)
    check(f'extract {phrase!r}', got == want, f'got {got!r}')

# The robot WRITES these to a face database. Enrolling someone as "Fine" is
# both wrong and tedious to undo, so a miss must beat a bad guess.
for phrase in ["i'm fine", "i'm good", "i'm not sure", "i'm here",
               "i'm just looking", "i'm ok"]:
    check(f'not a name: {phrase!r}', extract_name(phrase) is None,
          f'got {extract_name(phrase)!r}')

print('\n4. Vision starts the conversation - no wake word.')
d = Dialogue()
acts = d.on_face(None, 0.0)
check('unknown face is greeted', SAY in kinds(acts))
check('and the robot starts listening', value(acts, LISTEN) == 'on')
check('state is ASK_NAME', d.state == ASK_NAME)

acts = d.on_face('Alex', 100.0)
check('known face is NOT greeted mid-conversation', acts == [])

d2 = Dialogue()
acts = d2.on_face('Alex', 0.0)
check('known face greeted by name', 'Alex' in (value(acts, SAY) or ''))
acts = d2.on_face('Alex', 10.0)
check('and not greeted again immediately', acts == [])

print('\n5. A name is only enrolled after explicit confirmation.')
d = Dialogue()
d.on_face(None, 0.0)
acts = d.on_speech('MY NAME IS ALEX', 1.0)
check('asks for confirmation', 'Alex' in (value(acts, SAY) or ''))
check('does NOT enroll yet', ENROLL not in kinds(acts))
check('state is CONFIRM_NAME', d.state == CONFIRM_NAME)
acts = d.on_speech('YES', 2.0)
check('enrolls after yes', value(acts, ENROLL) == 'Alex')

d = Dialogue()
d.on_face(None, 0.0)
d.on_speech('MY NAME IS ALEX', 1.0)
acts = d.on_speech('NO', 2.0)
check('no -> asks again, does not enroll', ENROLL not in kinds(acts))
check('back to ASK_NAME', d.state == ASK_NAME)

d = Dialogue()
d.on_face(None, 0.0)
d.on_speech('MY NAME IS ALEX', 1.0)
acts = d.on_speech('NO MY NAME IS SAM', 2.0)
check('a corrected name is re-confirmed, not enrolled',
      ENROLL not in kinds(acts) and 'Sam' in (value(acts, SAY) or ''))

print('\n6. STOP works in every state.')
for build in (
    lambda: Dialogue(),
    lambda: (lambda d: (d.on_face(None, 0.0), d)[1])(Dialogue()),
    lambda: (lambda d: (d.on_face(None, 0.0), d.on_speech('MY NAME IS ALEX', 1.0), d)[2])(Dialogue()),
):
    d = build()
    before = d.state
    acts = d.on_speech('STOP', 5.0)
    check(f'stop from {before}', value(acts, SET_MODE) == 'stop')
    check(f'  and returns to idle from {before}', d.state == IDLE)

print('\n7. Commands do not need the conversation to be running.')
d = Dialogue()
d.on_face(None, 0.0)
acts = d.on_speech('FOLLOW ME', 1.0)
check('follow me sets mode', value(acts, SET_MODE) == 'follow')

print('\n8. Every waiting state times out.')
d = Dialogue(reply_timeout=5.0)
d.on_face(None, 0.0)
check('no timeout before the deadline', d.on_tick(3.0) == [])
acts = d.on_tick(99.0)
check('times out waiting for a name', SAY in kinds(acts))
check('and goes idle rather than waiting forever', d.state == IDLE)

print('\n9. It gives up politely rather than nagging.')
d = Dialogue(max_name_attempts=2)
d.on_face(None, 0.0)
d.on_speech('UHH WHAT', 1.0)
acts = d.on_speech('SOMETHING ELSE ENTIRELY', 2.0)
check('stops asking after max attempts', d.state == CHATTING, f'state={d.state}')

print('\n' + ('ALL PASS' if not fails else f'FAILURES: {fails}'))
sys.exit(1 if fails else 0)
