from screeninfo import get_monitors
import tkinter as tk


def getScreensInfo(screen):
    monitors = get_monitors()
    if len(monitors) > 1 and screen == 2:
        second_screen = monitors[1]
        return (
            second_screen.width,
            second_screen.height,
            second_screen.x,
            second_screen.y,
        )
    else:
        first_screen = monitors[0]
        return first_screen.width, first_screen.height, 0, 0


class VerticalScrolledFrame(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.configure(bg="")

        # Create a scrollbar
        vscrollbar = tk.Scrollbar(self, orient=tk.VERTICAL)
        vscrollbar.pack(fill=tk.Y, side=tk.RIGHT, expand=tk.FALSE)

        # Create a canvas within the frame
        canvas = tk.Canvas(
            self, bd=0, highlightthickness=0, yscrollcommand=vscrollbar.set
        )
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=tk.TRUE)

        vscrollbar.config(command=canvas.yview)

        # Reset the canvas view
        canvas.xview_moveto(0)
        canvas.yview_moveto(0)

        # Create an interior frame to be scrolled
        self.interior = interior = tk.Frame(canvas)
        interior_id = canvas.create_window(0, 0, window=interior, anchor=tk.NW)

        # Bind the canvas to the scrollbar
        def _configure_interior(event):
            size = (interior.winfo_reqwidth(), interior.winfo_reqheight())
            canvas.config(scrollregion="0 0 %s %s" % size)
            if interior.winfo_reqwidth() != canvas.winfo_width():
                canvas.config(width=interior.winfo_reqwidth())

        interior.bind("<Configure>", _configure_interior)

        def _configure_canvas(event):
            if interior.winfo_reqwidth() != canvas.winfo_width():
                canvas.itemconfigure(interior_id, width=canvas.winfo_width())

        canvas.bind("<Configure>", _configure_canvas)
