# Loads the full function list from the MATLAB website and outputs a syntax
# file snippet.
#
# This should be used with :r! inside of the matlab.vim syntax file.

import urllib, json, textwrap, re

# Skip any functions with these names, they're listed as MATLAB keywords
skip = [ 'return', 'function', 'switch', 'case', 'else', 'elseif', 'end',
        'if', 'otherwise', 'break', 'continue', 'do', 'for', 'while',
        'classdef', 'methods', 'properties', 'events', 'persistent', 'global',
        'try', 'catch', 'rethrow', 'throw', 'import', 'parfor', 'spmd' ]

# Skip things matched as constants
skip += [ 'eps', 'Inf', 'NaN', 'pi' ]

# Also skip the function "contains" since that's a special word in Vim :syntax
skip.append('contains')

def valid_item(item):
    # Methods and functions only
    if item['type'] not in [ "METHOD", "FUNCTION" ]:   return False
    # No special characters, except something in parentheses
    if not re.match(r'^\w+( \(.*\))?$', item['name']): return False
    # Nothing in the skip list
    if item['name'] in skip:                           return False
    return True

url = 'https://www.mathworks.com/help/search/reflist/doccenter/en/?type=function&listtype=alpha&product=matlab'

response = urllib.urlopen(url)
data = json.loads(response.read())

categories = data['category']['grouped-leaf-items']

function_names = set()
for cat in categories:
    items = cat['leaf-items']
    for item in items:
        if valid_item(item):
            name = re.sub(r'\s*\(.*\)', '', item['name'])
            function_names.add(name)

sorted_list = sorted(function_names, key=lambda f: f.lower())

prefix = 'syn keyword matlabFunc '
max_width = 78 - len(prefix)

sorted_list = textwrap.wrap(' '.join(sorted_list), max_width, break_long_words=False)
sorted_list = [ prefix + x for x in sorted_list ]

print "\n".join(sorted_list)

# Special case for "contains"
print prefix + "contains"
