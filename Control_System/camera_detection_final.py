# %% [markdown]
# <b> For Deployment on Raspberry Pi

# %%
import cv2
import numpy as np
import os
import time
import joblib
import tensorflow as tf
from collections import Counter
from picamera2 import Picamera2
from tensorflow.keras.applications import MobileNetV2
from tensorflow.keras.applications.mobilenet_v2 import preprocess_input
from tensorflow.keras.preprocessing.image import img_to_array
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA


def load_models(script_dir):
    """Loads TFLite, SVM, PCA, scaler, and class labels."""
    try:
        tflite_model_path = os.path.join(script_dir, "waste_classification_model.tflite")
        interpreter = tf.lite.Interpreter(model_path=tflite_model_path)
        interpreter.allocate_tensors()

        svm_classifier = joblib.load(os.path.join(script_dir, "svm_waste_classifier.joblib"))
        pca = joblib.load(os.path.join(script_dir, "pca.joblib"))
        scaler = joblib.load(os.path.join(script_dir, "scaler.joblib"))
        class_labels = joblib.load(os.path.join(script_dir, "class_labels.joblib"))

        feature_extractor = MobileNetV2(weights="imagenet", include_top=False, pooling="avg",
                                        input_shape=(160, 160, 3))

        return interpreter, svm_classifier, pca, scaler, class_labels, feature_extractor
    except Exception as e:
        raise RuntimeError(f"❌ Error loading models: {str(e)}")


def detect_object(picam2):

    script_dir = os.getcwd()
    interpreter, svm_classifier, pca, scaler, class_labels, feature_extractor = load_models(script_dir)

    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    input_shape = input_details[0]['shape'][1:3]

    prev_frame = None
    
    def detect_motion(image):
        """Detect motion using frame differencing."""
        nonlocal prev_frame
        if prev_frame is None:
            prev_frame = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
            return False
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        diff = cv2.absdiff(prev_frame, gray)
        prev_frame = gray
        return np.sum(diff) > 60000
    
    def classify_waste_with_tflite(frame):
        """Classifies waste using TensorFlow Lite."""
        try:
            image = cv2.resize(frame, tuple(input_shape)) / 255.0
            image = np.expand_dims(img_to_array(image), axis=0).astype(np.float32)
            interpreter.set_tensor(input_details[0]['index'], image)
            interpreter.invoke()
            predictions = interpreter.get_tensor(output_details[0]['index'])
            return class_labels[np.argmax(predictions)], np.max(predictions)
        except Exception as e:
            print(f"⚠️ TFLite classification error: {str(e)}")
            return "others", 0.0

    def classify_waste_with_svm(frame):
        """Classifies waste using the SVM model."""
        try:
            image = cv2.resize(frame, (160, 160))
            image = np.expand_dims(img_to_array(image), axis=0)
            image = preprocess_input(image)
            features = feature_extractor.predict(image).flatten()

            expected_features = scaler.n_features_in_
            if len(features) != expected_features:
                print(f"⚠️ Feature shape mismatch! Expected {expected_features}, got {len(features)}")
                return "others", 0.0

            feature_scaled = scaler.transform([features])
            feature_pca = pca.transform(feature_scaled)
            probabilities = svm_classifier.predict_proba(feature_pca)[0]

            return class_labels[np.argmax(probabilities)], np.max(probabilities)
        except Exception as e:
            print(f"⚠️ SVM classification error: {str(e)}")
            return "others", 0.0

    while True:
        image = picam2.capture_array()
        if detect_motion(image):
            classification_results = []
            for _ in range(3):  # More reliable classification with 3 snapshots
                tflite_label, tflite_confidence = classify_waste_with_tflite(image)
                svm_label, svm_confidence = classify_waste_with_svm(image)
                final_label = tflite_label if tflite_confidence >= svm_confidence else svm_label
                final_confidence = max(tflite_confidence, svm_confidence)
                classification_results.append((final_label, final_confidence))
                time.sleep(0.1)

            most_common_class, _ = Counter([c[0] for c in classification_results]).most_common(1)[0]
            avg_confidence = np.mean([c[1] for c in classification_results if c[0] == most_common_class])

            # ✅ Show the final classified image with the predicted label
            cv2.putText(image, f"Class: {most_common_class} ({avg_confidence:.2f})", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("Classified Image - Showing for 5 Seconds", image)
            cv2.waitKey(5000)  # ✅ Display for 5 seconds (5000 ms)
            cv2.destroyAllWindows()  # ✅ Close the viewer after display

            if most_common_class == "metal" and avg_confidence > 0.7:
                return "metal"
            elif most_common_class == "paper" and avg_confidence > 0.7:
                return "paper"
            elif most_common_class == "plastic" and avg_confidence > 0.55:
                return "plastic"
            else:
                return "others"

# ✅ Initialize Picamera2 outside the function
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (320, 240), "format": "RGB888"})
picam2.configure(config)
picam2.start()
time.sleep(2)

try:
    detected_class = detect_object(picam2)
    print(f"\n✅ Detected Waste Category: {detected_class}")
except Exception as e:
    print(f"❌ Error during detection: {str(e)}")
finally:
    picam2.stop()
    picam2.close()
    print("✅ Camera shut down successfully.")


