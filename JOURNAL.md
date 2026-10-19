---
title: "Spiral Marble Run"
author: "YOUR NAME"
description: "A motorised marble machine: Archimedes screw lift, spinning funnel, Plinko board and a long switchback track."
created_at: "YYYY-MM-DD"
---

# Build Journal

**Total time: __ h**

<!--
How to use this file:
- Add one entry per work session, newest at the bottom.
- Write the date, the hours you actually spent, what you did and what went wrong.
- Add at least one picture per entry (a screenshot, photo or render). Put images in docs/journal/.
- Hack Club reviewers read this to check your time, so be honest and specific.
-->

## YYYY-MM-DD: Idea and first CAD (__ h)

- What I wanted to build and why
- First Fusion model: basic plinko, funnel, tube track and an enclosed screw lift
- Problems found: the heights didn't add up (the lift was too short for the drop), and an 8 mm marble didn't fit between a 4 mm shaft and a 16 mm tube

![first model](docs/render_front.jpg)

## YYYY-MM-DD: Redesign: open track and rail screw (__ h)

- Switched to an open U-channel track with switchbacks (longer run, about 770 mm)
- Changed the lift to an open screw plus a guide rail, tilted 8°, with a 10 mm core so the marble can't slide inward
- Chute now drops the marble onto the funnel edge so it spins
- New Plinko board: 40° tray, bar pegs, sawtooth walls, two exits
- Added the interference and marble-path checks to the script

## YYYY-MM-DD: Firmware (__ h)

- XIAO ESP32-C3 + DRV8833 driving the N20
- Soft start/stop, long-press un-jam, speed pot, auto-off

## YYYY-MM-DD: Printing (__ h)

- Print settings, failed prints, fixes

## YYYY-MM-DD: Assembly and testing (__ h)

- What worked first time, what needed sanding or tuning
- Video of it running: (link)
