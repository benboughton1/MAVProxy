import datetime


class Text():
    '''text to write to console'''
    def __init__(self, text, fg='black', bg='white'):
        self.text = text
        self.fg = fg
        self.bg = bg

class Value():
    '''a value for the status bar'''
    def __init__(self, name, text, row=0, fg='black', bg='white'):
        self.name = name
        self.text = text
        self.row = row
        self.fg = fg
        self.bg = bg


class ValueDict():
    def __init__(self, values):
        self.values = values


class TextList():
    def __init__(self):
        self.text = []

    def add(self, line):
        line["time"] = datetime.datetime.now().isoformat()
        self.text.append(line)
        # trim to 100 lines
        self.text = self.text[-100:]
