
class Indicator {
public:
    // Constructor
    Indicator(int pin) : Pin_(pin), State_(false), previousMillis_(0),
    blinkCodeIndex_(0), blinkState_(IDLE), blinkStartTime_(0) {
        pinMode(Pin_, OUTPUT);
    }

    // Toggle the LED state
    bool ToggleIndicator() {
        State_ = !State_;
        digitalWrite(Pin_, State_);
        return State_;
    }

    // Set the LED state explicitly
    void SetState(bool newState) {
        State_ = newState;
        digitalWrite(Pin_, State_);
    }

    // Blink the LED at a fixed frequency
    void Blink(unsigned int frequencyMs) {
        unsigned int currentMillis = millis();

        if (currentMillis - previousMillis_ >= frequencyMs) {
            previousMillis_ = currentMillis;
            ToggleIndicator();
        }
    }

    // Blink a specific pattern (e.g., "SLS" for Short-Long-Short)
    void BlinkCode(const char* blinkCode) {
        unsigned int currentMillis = millis();

        switch (blinkState_) {
            case IDLE:
                // Start the blink sequence immediately
                blinkCodeIndex_ = 0;
                blinkState_ = BLINK_ON;
                break;

            case BLINK_ON:
                // Turn on the LED and start the blink duration
                if (blinkCodeIndex_ < 4 && blinkCode[blinkCodeIndex_] != '\0') {
                    SetState(true);
                    blinkStartTime_ = currentMillis;
                    blinkState_ = BLINK_WAIT;
                } else {
                    // End of sequence, wait for the end gap before repeating
                    blinkState_ = END_GAP;
                    blinkStartTime_ = currentMillis;
                }
                break;

            case BLINK_WAIT:
                // Wait for the blink duration to complete
                if (currentMillis - blinkStartTime_ >= getBlinkDuration(blinkCode[blinkCodeIndex_])) {
                    SetState(false);
                    blinkCodeIndex_++;
                    blinkState_ = BLINK_GAP;
                    blinkStartTime_ = currentMillis;
                }
                break;

            case BLINK_GAP:
                // Wait for the gap between blinks
                if (currentMillis - blinkStartTime_ >= BLINK_GAP_DURATION) {
                    blinkState_ = BLINK_ON;
                }
                break;

            case END_GAP:
                // Wait for the end gap after the sequence ends
                if (currentMillis - blinkStartTime_ >= END_GAP_DURATION) {
                    blinkState_ = IDLE; // Restart the sequence
                }
                break;
        }
    }

private:
    // States for the blink code state machine
    enum BlinkState {
        IDLE,      // Waiting to start the blink sequence
        BLINK_ON,  // LED is on
        BLINK_WAIT, // Waiting for the blink duration to complete
        BLINK_GAP,  // Waiting for the gap between blinks
        END_GAP    // Waiting for the end gap after the sequence ends
    };

    int Pin_; // Output Pin
    bool State_; // Current LED State
    unsigned int previousMillis_; // For the Blink method
    unsigned char blinkCodeIndex_; // Current position in the blink code
    BlinkState blinkState_; // Current state of the blink sequence
    unsigned int blinkStartTime_; // When the current blink started

    // Helper function to get the duration of a blink based on the code character
    unsigned int getBlinkDuration(char blinkChar) {
        return (blinkChar == 'L') ? LONG_BLINK_DURATION : SHORT_BLINK_DURATION;
    }
};
