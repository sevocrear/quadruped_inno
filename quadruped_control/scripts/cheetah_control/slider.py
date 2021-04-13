import wx

class MyForm(wx.Frame):

    def __init__(self):
        wx.Frame.__init__(self, None, wx.ID_ANY, "Timer Tutorial 1", 
                                   size=(500,500))

        # Add a panel so it looks the correct on all platforms
        panel = wx.Panel(self, wx.ID_ANY)

        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.update, self.timer)

        SpeedOfSimulation = wx.Slider(p, pos=(800,10), size=(200,100), minValue=0, maxValue=1000)
        SpeedOfSimulation.Bind(wx.EVT_SCROLL, SetRate)
        self.SpeedOfSimulation = SpeedOfSimulation

    def update(self, event):

      # Compute all forces on all stars

        SpeedOfSimulation = self.SpeedOfSimulation.GetValue()