#!/usr/bin/env python3
"""
Turn a speech transcript into an intent.

Deliberately NOT an LLM. Three reasons:

  * SAFETY. "stop" must work when the wifi is down, the cloud is slow, or the
    model is loaded. It is pattern matching on a string and it never leaves the
    robot. Routing it through a language model would make the most important
    command the least reliable one.
  * SPEED. A match is microseconds. An LLM round trip is hundreds of
    milliseconds at best.
  * It cannot hallucinate. An LLM asked to extract a name will occasionally
    invent one, and this robot WRITES that name to a face database.

Input is raw recogniser output, which on this stack is UPPERCASE with no
punctuation ("MY NAME IS ALEX"), so everything is normalised first.
"""
import re
from dataclasses import dataclass, field
from typing import Optional

STOP = 'stop'
FOLLOW = 'follow'
COME = 'come'
WANDER = 'wander'
NAME = 'name'
YES = 'yes'
NO = 'no'
CHAT = 'chat'


@dataclass
class Intent:
    kind: str
    text: str = ''
    slots: dict = field(default_factory=dict)
    confidence: float = 1.0


def normalise(s: str) -> str:
    """Lowercase, strip punctuation, collapse whitespace."""
    s = (s or '').lower()
    s = re.sub(r"[^a-z0-9' ]+", ' ', s)
    return re.sub(r'\s+', ' ', s).strip()


# Stop is matched LOOSELY and FIRST. A false positive stops a slow robot, which
# is harmless. A false negative means it keeps going when someone asked it not
# to, which is not.
_STOP = re.compile(r'\b(stop|halt|wait|freeze|hold on|stay|stop it|stop there)\b')

_FOLLOW = re.compile(r'\b(follow me|follow|come with me|come along|this way)\b')
_COME = re.compile(r'\b(come here|come to me|over here)\b')
_WANDER = re.compile(r'\b(wander|explore|look around|go explore|roam)\b')

_YES = re.compile(r"\b(yes|yeah|yep|yup|correct|that's right|thats right|right|"
                  r"sure|affirmative|exactly|uh huh)\b")
_NO = re.compile(r"\b(no|nope|nah|wrong|incorrect|not quite|that's wrong|"
                 r"thats wrong)\b")

# Name capture. Ordered: the most explicit phrasing wins, so "my name is Sam"
# is not matched by the looser "it's Sam" rule.
_NAME_PATTERNS = [
    re.compile(r"\bmy name is ([a-z']+(?: [a-z']+)?)"),
    re.compile(r"\bi am called ([a-z']+(?: [a-z']+)?)"),
    re.compile(r"\byou can call me ([a-z']+(?: [a-z']+)?)"),
    re.compile(r"\bcall me ([a-z']+(?: [a-z']+)?)"),
    re.compile(r"\bi'm ([a-z']+)"),
    re.compile(r"\bi am ([a-z']+)"),
    re.compile(r"\bthis is ([a-z']+)"),
    re.compile(r"\bit's ([a-z']+)"),
    re.compile(r"\bits ([a-z']+)"),
]

# Words that follow "I'm ..." but are plainly not names. Without this the robot
# enrols a face as "fine" or "good", which is both wrong and very hard to undo
# once it is in the database.
_NOT_NAMES = {
    'fine', 'good', 'great', 'ok', 'okay', 'well', 'here', 'back', 'sorry',
    'not', 'just', 'going', 'doing', 'tired', 'busy', 'hungry', 'happy',
    'sure', 'ready', 'listening', 'talking', 'a', 'an', 'the', 'no', 'yes',
    'from', 'in', 'on', 'at', 'with', 'about', 'really', 'very', 'so',
}


def extract_name(text: str) -> Optional[str]:
    """
    Pull a person's name out of a reply, or None.

    Returning None is a perfectly good outcome - the caller should ask again
    rather than guess. Enrolling the wrong name is worse than asking twice.
    """
    t = normalise(text)
    for pat in _NAME_PATTERNS:
        m = pat.search(t)
        if not m:
            continue
        cand = m.group(1).strip()
        first = cand.split()[0] if cand else ''
        if not first or first in _NOT_NAMES:
            continue
        if len(first) < 2:
            continue
        return ' '.join(w.capitalize() for w in cand.split())
    return None


def classify(text: str) -> Intent:
    """
    Classify a transcript. Safety commands are checked before anything else.
    """
    t = normalise(text)
    if not t:
        return Intent(CHAT, text='')

    if _STOP.search(t):
        return Intent(STOP, text=t)
    if _FOLLOW.search(t):
        return Intent(FOLLOW, text=t)
    if _COME.search(t):
        return Intent(COME, text=t)
    if _WANDER.search(t):
        return Intent(WANDER, text=t)

    name = extract_name(t)
    if name:
        return Intent(NAME, text=t, slots={'name': name})

    # Bare yes/no only counts when the utterance is essentially just that;
    # otherwise "no, my name is Sam" would read as a refusal.
    if _YES.search(t) and len(t.split()) <= 3:
        return Intent(YES, text=t)
    if _NO.search(t) and len(t.split()) <= 3:
        return Intent(NO, text=t)

    return Intent(CHAT, text=t)
