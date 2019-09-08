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
In this post, I assume that you are familiar with basic score notation.

{{< sheetmusic file="sheetmusic/c-major-scale.xml" >}}

Harmony is basically a sequences of chords and responsible for giving the music a sense of color and direction.
For example: Major, minor chords.
However, a chord played on its own doesn't convey much musical sense.
Pieces from the classical as well as jazz era sound interesting because they have an interesting sequences of chords.
You cannot expect to play random chords and create a nice sounding piece.
Therefore, there must be some underlying structure in the harmony that makes certain sequences pleasant to listen to.

# Functional Harmony (TODO)

In his paper [Towards a generative syntax of tonal harmony](https://www.tandfonline.com/doi/pdf/10.1080/17459737.2011.573676?needAccess=true),
Martin Rohrmeier created a formal framework which uses context free grammar rules to generate chord sequences.
While his method is meant to generalize to all kinds of musical styles, I found it especially applicable to explain the harmoic structure of jazz standards.

- Concepts of Preparation and Prolongation
- Grammar rules
- showing tree

