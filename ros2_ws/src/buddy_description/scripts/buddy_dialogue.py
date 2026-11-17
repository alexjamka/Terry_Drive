#!/usr/bin/env python3
"""
Buddy's conversation state machine.

Pure logic: it takes events and returns actions. No ROS, no audio, no network,
no clock of its own - the caller passes `now`. That makes the whole behaviour
unit-testable without hardware, which matters because the alternative is
debugging a dialogue by talking to a robot.

The goal behaviour: while roaming, the robot sees a face it does not recognise,
greets them unprompted, asks their name, confirms it, and remembers it. No wake
word - VISION starts the conversation, not a spoken trigger.

    face_seen(unknown) -> ASK_NAME -> (hears a name) -> CONFIRM_NAME
                                                            |
                                           yes -> ENROLL -> CHATTING
                                           no  -> ASK_NAME (retry)

Design rules that are not negotiable:
  * STOP is handled in EVERY state, before anything else.
  * A name is only written to the face database after explicit confirmation.
    Enrolling the wrong name is hard to undo and the robot then greets someone
    by a stranger's name forever.
  * Every waiting state times out. A robot stuck waiting for a reply that never
    comes is a robot that has stopped working, silently.
"""
from dataclasses import dataclass, field
from typing import List, Optional

from buddy_intents import (CHAT, COME, FOLLOW, NAME, NO, STOP, WANDER, YES,
                           Intent, classify)

IDLE = 'IDLE'
ASK_NAME = 'ASK_NAME'
CONFIRM_NAME = 'CONFIRM_NAME'
CHATTING = 'CHATTING'

SAY = 'say'
ENROLL = 'enroll'
SET_MODE = 'set_mode'
LISTEN = 'listen'


@dataclass
class Action:
    kind: str
    value: str = ''


@dataclass
class Dialogue:
    # How long before the robot gives up waiting for a spoken reply.
    reply_timeout: float = 12.0
    # Don't greet the same person again within this many seconds.
    greet_cooldown: float = 300.0
    # Give up asking for a name after this many tries, politely.
    max_name_attempts: int = 2
    # Drop out of conversation after this long with nothing said.
    chat_timeout: float = 45.0

    state: str = IDLE
    pending_name: Optional[str] = None
    current_person: Optional[str] = None
    attempts: int = 0
    _state_since: float = 0.0
    _last_greeted: dict = field(default_factory=dict)

    # ----------------------------------------------------------------- helpers
    def _go(self, state: str, now: float) -> None:
        self.state = state
        self._state_since = now

    def _elapsed(self, now: float) -> float:
        return now - self._state_since

    # ------------------------------------------------------------------ events
    def on_face(self, name: Optional[str], now: float) -> List[Action]:
        """
        A face was seen. `name` is None when it is not in the database.

        Ignored unless idle: being interrupted mid-sentence by a second face
        would make the robot abandon the person it is already talking to.
        """
        if self.state != IDLE:
            return []

        key = name or '__unknown__'
        last = self._last_greeted.get(key)
        if last is not None and now - last < self.greet_cooldown:
            return []
        self._last_greeted[key] = now

        if name:
            self.current_person = name
            self._go(CHATTING, now)
            return [Action(SAY, f'Hello again, {name}.'), Action(LISTEN, 'on')]

        self.attempts = 1
        self.current_person = None
        self._go(ASK_NAME, now)
        return [Action(SAY, "Hello. I don't think we've met. What's your name?"),
                Action(LISTEN, 'on')]

    def on_speech(self, transcript: str, now: float) -> List[Action]:
        intent = classify(transcript)

        # Safety first, in every state, before any dialogue logic.
        if intent.kind == STOP:
            self._go(IDLE, now)
            self.pending_name = None
            return [Action(SET_MODE, 'stop'), Action(SAY, 'Stopping.'),
                    Action(LISTEN, 'off')]

        if intent.kind == FOLLOW or intent.kind == COME:
            self._go(CHATTING, now)
            return [Action(SET_MODE, 'follow'), Action(SAY, 'Okay, following you.')]
        if intent.kind == WANDER:
            self._go(IDLE, now)
            return [Action(SET_MODE, 'wander'), Action(SAY, 'Okay, exploring.'),
                    Action(LISTEN, 'off')]

        if self.state == ASK_NAME:
            return self._on_speech_ask_name(intent, now)
        if self.state == CONFIRM_NAME:
            return self._on_speech_confirm(intent, now)
        if self.state == CHATTING:
            self._state_since = now
            return [Action(CHAT, intent.text)] if intent.text else []
        return []

    def _on_speech_ask_name(self, intent: Intent, now: float) -> List[Action]:
        if intent.kind == NAME:
            self.pending_name = intent.slots['name']
            self._go(CONFIRM_NAME, now)
            return [Action(SAY, f'Did you say {self.pending_name}?')]

        # Heard something, but no name in it.
        self.attempts += 1
        if self.attempts > self.max_name_attempts:
            self._go(CHATTING, now)
            return [Action(SAY, "That's alright, you can tell me another time.")]
        self._state_since = now
        return [Action(SAY, "Sorry, I didn't catch your name. What should I call you?")]

    def _on_speech_confirm(self, intent: Intent, now: float) -> List[Action]:
        if intent.kind == YES:
            name = self.pending_name
            self.current_person = name
            self.pending_name = None
            self._go(CHATTING, now)
            return [Action(ENROLL, name),
                    Action(SAY, f"Nice to meet you, {name}. I'll remember you.")]

        if intent.kind == NO:
            self.pending_name = None
            self.attempts += 1
            if self.attempts > self.max_name_attempts:
                self._go(CHATTING, now)
                return [Action(SAY, "No matter, we can sort that out later.")]
            self._go(ASK_NAME, now)
            return [Action(SAY, 'Sorry. What should I call you?')]

        # They said something else entirely - treat a fresh name as a correction.
        if intent.kind == NAME:
            self.pending_name = intent.slots['name']
            self._state_since = now
            return [Action(SAY, f'Did you say {self.pending_name}?')]

        self._state_since = now
        return [Action(SAY, f'Sorry, is {self.pending_name} right?')]

    def on_tick(self, now: float) -> List[Action]:
        """Timeouts. Without these the robot waits forever and looks broken."""
        if self.state in (ASK_NAME, CONFIRM_NAME):
            if self._elapsed(now) > self.reply_timeout:
                self.pending_name = None
                self._go(IDLE, now)
                return [Action(SAY, 'Maybe another time.'), Action(LISTEN, 'off')]
        elif self.state == CHATTING:
            if self._elapsed(now) > self.chat_timeout:
                self._go(IDLE, now)
                return [Action(LISTEN, 'off')]
        return []
