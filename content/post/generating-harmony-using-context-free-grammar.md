---
title: "Generating Harmony using Context Free Grammar"
description: "Explaining a Framework for Computational Music Generation"
author:
  name: "Philipp Badenhoop"
date: 2019-09-06
draft: true
tags:
- Music Theory
- Context Free Grammar
markup: mmark
use: [math, sheetmusic]
---

# Introduction

In this post, I will explain an exciting framework used to analyze and generate sequences of chords in musical pieces. 

## Background

Playing the piano has always been one of my greatest passions.
After playing classical music for several years, I fell in love with improvisation as well as playing songs in different arrangements and styles.
However, compared to playing songs from scores, these skills require some knowledge of know the music actually works.
At its core, we can divide music into three fundamental parts, namely **melody**, **rhythm** and **harmony**.
While you surely have an intuitive understanding of what melody and rhythm is, you may not be quite familiar with the last one.
We can basically put it like that: 
Melody questions which note is played after the other while harmony describes which notes can be played simultaneously.
In western music, harmony is represented as a sequence of chords which in turn are notes that are picked from a certain scale.

The simplest form of a chord is the **triad** which consists of two stacked third intervals. 
For example, the C major chord contains the notes *c*, *e* and *g* which are derived by going a major third (four semitones) from *c* to *e* and a minor third (three semitones) from *e* to *g*.
Similarily, we can define the three remaining types of triads:

{.table .table-striped}
type | first interval | second interval | example notation | example notes
--- | --- | --- | --- | ---
major triad | major third | minor third | *C* | *c*, *e*, *g*
minor triad | minor third | major third | *Cm* | *c*, *e♭*, *g*
diminished triad | minor third | minor third | *Cdim* | *c*, *e♭*, *g♭*
augmented triad | major third | major third | *Caug* | *c*, *e*, *g♯*

{{< sheetmusic file="sheetmusic/c-triads.xml" mode="simple" >}}

Grab your instrument and play these chords to get a feeling of what they sound like.
You might notice that the major triad sounds quite happy compared to the minor and the even darker diminished triad.
We can extend this concept by stacking another third on top of these triads.
The resulting four-note chords are then called **seventh chords**.
At this point, all thirds we stack on top of the seventh chords are called **extensions** or **color tones**.
The resulting **nineth**, **eleventh** and **thirteenth** chords are especially used in jazz music and may sound very spicy.

The whole point of the theory of harmony is to put chords into a musical context.
Most commonly in western music, this context is induced by the major or minor scale.
A scale is just a set of notes that follow a specific interval pattern.
For example, a major scale has seven notes which stack up in the following (repeating) interval sequence:

root, whole step, whole step, half step, whole step, whole step, whole step, half step and now we're back at the root.

In music notation, the following represents a C major scale:

{{< sheetmusic file="sheetmusic/c-major-scale.xml" mode="simple" >}}

The minor scale follows a different pattern:
root, whole step, half step, whole step, whole step, half step, whole step, whole step.
A C minor scale looks like this:

{{< sheetmusic file="sheetmusic/c-minor-scale.xml" mode="simple" >}}

Now, we can build so called **diatonic** chords from a scale.
That means that we choose one of the notes in our scale as the root note and stack up thirds such that all resulting notes of our chord are also contained inside the scale.
Let's say we're in the key of C major.
A diatonic triad with the root *e* is a *Em* consisting of the notes *e*, *g* and *h*.
Likewise, a diatonic triad with root *b* is a *Bdim* with notes *b*, *d* and *f*.
The root note of a particular diatonic chord indicates its **degree** which we denote with roman numbers.
For example, any chord that builds on F has degree IV because its the fourth chord in the C major scale.
By convention, upper case roman numbers refer to major chords, while minor and diminished chords use lower case roman numbers.
All diatonic triads over a C major scale with their respective degree are shown below:

{{< sheetmusic file="sheetmusic/c-major-scale-triads.xml" mode="simple" >}}

Finally, we could go ahead and start making some music by playing any diatonic chords over a fixed major or minor scale.
In fact, if you want to start a career as a pop musician, you may already learned enough theory to produce your four-chord-songs.
However, chord progressions in jazz and classical music are more complex and require a more thorough understanding of how the music works.

# Functional Harmony (TODO)

In his paper [Towards a generative syntax of tonal harmony](https://www.tandfonline.com/doi/pdf/10.1080/17459737.2011.573676?needAccess=true),
Martin Rohrmeier created a formal framework which uses context free grammar rules to generate chord sequences.
While his method is meant to generalize to all kinds of musical styles, I found it especially applicable to explain the harmoic structure of jazz standards.

- Concepts of Preparation and Prolongation
- Grammar rules
- showing tree

