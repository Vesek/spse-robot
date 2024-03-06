from nicegui import ui
import cv2
import numpy as np
from src.webcam import Camera

camera = Camera()

with ui.header().style('background-color: #262626;'):
    ui.label('SPSE-Robot control panel').classes("text-5xl")

with ui.card().classes('tight absolute-center no-shadow border-[1px] w-3/4'):
    with ui.row(wrap=False).classes("p-2 w-full justify-center"):
        with ui.card().classes('p-2 no-shadow border-[1px] rounded'):
            p_label = ui.label()
            p_label.classes("mx-auto text-3xl")
        with ui.card().classes('p-2 no-shadow border-[1px] rounded'):
            k_label = ui.label()
            k_label.classes("mx-auto text-3xl")
    with ui.row(wrap=False).classes("p-4 w-full"):
        p_slider = ui.slider(min=0, max=2, step=0.01, value=1)
        p_slider.bind_value_to(p_label, "text", forward=lambda n: f'P: {n:.2f}')
    with ui.row(wrap=False).classes("p-4 w-full"):
        k_slider = ui.slider(min=0, max=2, step=0.01, value=1)
        k_slider.bind_value_to(k_label, "text", forward=lambda n: f'K: {n:.2f}')
    with ui.row(wrap=False).classes("p-3 w-full justify-center"):
        ui.button('Download', on_click=lambda:ui.download(cv2.imencode(".png",camera.capture())[1].tobytes(), 'hello.png'))

ui.run(dark=True,favicon="https://www.spseplzen.cz/wp-content/uploads/2017/09/logo_1.png",title="SPSE-Robot")