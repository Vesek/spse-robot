import cv2
import numpy as np
from src.analyzer import Analyzer

# Flags (maybe will add as arguments)
running_on_rpi = True
headless = False
in_fb = True
perf_metrics = False

if running_on_rpi:
    from src.picam import Camera
    from src.motors import Motors
    motors = Motors()
    motors.enable()
else:
    from src.webcam import Camera

camera = Camera()
analyzer = Analyzer(perf_metrics)

try:
    while True:
        frame = camera.capture()

        preprocessed_frame = analyzer.preprocessing(frame)

        lines = analyzer.detect_lines(preprocessed_frame)

        line_image = np.copy(frame)

        theta_avg, line_image = analyzer.process_lines(lines, line_image)
        if theta_avg is not None:
            print(theta_avg)

        if not headless:
            if in_fb:
                frame32 = cv2.cvtColor(line_image, cv2.COLOR_BGR2BGRA)
                fbframe = cv2.resize(frame32, (1920,1080))
                with open('/dev/fb0', 'rb+') as buf:
                    buf.write(fbframe)
            else:
                cv2.imshow('video', line_image)
                if cv2.waitKey(1) == 27:
                    break
        
        if perf_metrics: print(analyzer.times)
except KeyboardInterrupt:
    print("\nQuitting because of a keyboard interrupt\n")
    motors.deinit()
    camera.deinit()
    if not headless and not in_fb: cv2.destroyAllWindows()
