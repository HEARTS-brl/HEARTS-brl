# Filename: gcp_keywords.py
# Created: 15 Aug 2017
# Author : Derek Ripper
# Purpose: Define list of keywords/phrases that can be used in
#          the google_cloud speech recognition in s2t.py to be 
#          used as the "preferred_phrases"            


def gcp_keywords():
    preferred_phrases = [ \
    "no such way here",
#    "jangle room",
    "go to the jangle room"
    ]

    return preferred_phrases
