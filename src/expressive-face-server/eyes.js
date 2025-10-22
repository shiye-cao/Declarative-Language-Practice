/**
 * @fileoverview
 * Defines the EyeController class to manage and animate facial expressions on a webpage.
 * This file handles the animation of eyes and mouth based on real-time events received
 * through Socket.IO from the server.
 *
 * @requires anime: Anime.js library used for smooth animation effects.
 */

class EyeController {
    /**
     * Constructs an instance of the EyeController with specific DOM elements.
     * @param {Object} elements - DOM elements for the face, eyes, eyelids, and mouth.
     */
    constructor(elements = {}) {
        this.elements = elements;
        this.currentEmotion = "happy"; // First emotion
        this.previousEmotion = "neutral"; // Default emotion
    }

    /**
     * Sets the previous emotion to the current emotion.
     * @param {string} emotion - The emotion to be saved as previousEmotion.
     */
    setCurrentEmotion(emotion) {
        console.log(emotion)
        this.currentEmotion = emotion;
    }

    /**
     * Sets the previous emotion to the current emotion.
     * @param {string} emotion - The emotion to be saved as previousEmotion.
     */
    setPreviousEmotion(emotion) {
        this.previousEmotion = emotion;
    }

    /**
     * Animates a property of a target element.
     * @param {Array | Object} target - DOM elements to be animated.
     * @param {string} property - CSS property to animate.
     * @param {Array | number | string} value - End value(s) of the animation.
     * @param {number} duration - Duration of the animation in milliseconds.
     */
    animateProperty(target, property, value, duration = 1000) {
        anime({
            targets: target,
            [property]: value,
            duration: duration,
            easing: "easeInOutQuad",
        });
    }

    /**
     * Triggers a blink animation on the eyes.
     * @param {number} duration - Duration of the blink animation in milliseconds.
     */
    blink(duration = 150) {
        this.animateProperty(
            [this.elements.leftEye, this.elements.rightEye],
            "scaleY",
            [1, 0, 1],
            duration
        );
    }

    /**
     * Starts an infinite blinking loop for the eyes.
     */
    startBlinking() {
        this.blinkingTimeline = anime.timeline({
            easing: "easeInOutQuad",
            loop: true,
            duration: 2500,
        });

        this.blinkingTimeline.add(
            {
                targets: [this.elements.leftEye, this.elements.rightEye],
                scaleY: [1, 0, 1],
                duration: 600,
            },
            1700
        );
    }

    /**
     * Stops the blinking animation and resets the eyes to the original state.
     */
    stopBlinking() {
        if (this.blinkingTimeline) {
            this.blinkingTimeline.pause();
            this.blinkingTimeline = null;
            anime({
                targets: [this.elements.leftEye, this.elements.rightEye],
                scaleY: 1,
                duration: 500,
                easing: "easeOutQuad",
            });
        }
    }

    /**
     * Resets all facial features to their original state, stopping all ongoing animations.
     * Future work: Separate resetting of facial features and stopping of individual animations.
     */
    resetToOriginalState() {
        this.setCurrentEmotion("neutral");
        this.stopTalking();
        const timeline = anime.timeline({
            easing: "easeOutQuad",
            duration: 500,
        });

        timeline
            .add({
                targets: [this.elements.leftEye, this.elements.rightEye],
                backgroundColor: "#000000",
                scaleX: 1,
                scaleY: 1,
                scale: 1,
                translateX: "0%",
                translateY: "0%",
                borderRadius: "50%",
                duration: 500,
                easing: "easeOutQuad",
            })
            .add(
                {
                    targets: [
                        this.elements.upperLeftEyelid,
                        this.elements.upperRightEyelid,
                        this.elements.lowerRightEyelid,
                        this.elements.lowerLeftEyelid,
                    ],
                    translateY: "0%",
                    rotate: "0deg",
                    duration: 500,
                    easing: "easeInOutSine",
                },
                0
            )
            .add({
                targets: this.elements.mouth,
                backgroundColor: "#ffc0cb",
                scaleX: 0.8,
                scaleY: 0.6,
                scale: 1,
                width: "30vmin",
                height: "15vmin",
                bottom: "50px",
                translateX: "0%",
                translateY: "0%",
                borderRadius: "50%",
                rotate: "0deg",
                duration: 500,
                easing: "easeOutQuad",
            });
    }

    /**
     * Execute the stored emotion expression.
     * @param {string} emotion - The emotion to express.
     */
    executeExpression(emotion) {
        console.log("execute now:", emotion);
        switch (emotion) {
            case "neutral":
                this.expressNeutral();
                break;
            case "excited":
                this.expressExcited();
                break;
            case "happy":
                this.expressHappiness();
                break;
            case "surprised":
                this.expressSurprised();
                break;
            case "interested":
                this.expressInterested();
                break;
            case "thinking":
                this.expressConfused();
                break;
            case "satisfied":
                this.expressSatisfied();
                break;
            default:
                console.log("Unknown emotion command:", emotion);
                break;
        }
    }

    expressNeutral() {
        this.resetToOriginalState();
        this.setCurrentEmotion("neutral");
        const timeline = anime.timeline({
            easing: "easeOutQuad",
            duration: 1500,
        });

        timeline
            .add({
                targets: this.elements.mouth,
                borderRadius: "50%",
                scaleX: 0.8,
                scaleY: 0.6,
                scale: 1,
                width: "30vmin",
                height: "15vmin",
                bottom: "50px",
                translateX: "0%",
                translateY: "0%",
                rotate: "0deg",
                easing: "easeOutQuad",
            },
                0);
    }

    /**
     * Implementation of a talking animation
     *  Future work: Add random variations to range of scaling, etc for a more dynamic feeling
     */
    startTalking() {
        if (this.talkingTimeline) {
            return;
        }
        else {
            // Save the current emotion before starting the talking animation
            this.setPreviousEmotion(this.currentEmotion || "neutral");
        }
        this.talkingTimeline = anime.timeline({
            easing: "easeInOutQuad",
            loop: true,
            duration: 200,
        });

        this.talkingTimeline
            .add({
                targets: this.elements.mouth,
                scaleY: [1, 1.2],
                scaleX: [1, 1.1],
                duration: 200,
                easing: "easeInOutSine",
            })
            .add({
                targets: this.elements.mouth,
                scaleY: 1,
                scaleX: 1,
                duration: 200,
                easing: "easeInOutSine",
            })
            .add({
                targets: this.elements.mouth,
                scaleY: [1, 1.3],
                scaleX: [1, 1.15],
                duration: 300,
                easing: "easeInOutSine",
            })
            .add({
                targets: this.elements.mouth,
                scaleY: 1,
                scaleX: 1,
                duration: 300,
                easing: "easeInOutSine",
            });
    }

    /**
     * Function to terminate the talking animation
     */
    stopTalking() {
        if (this.talkingTimeline) {
            console.log("stopTalking now:");
            this.talkingTimeline.pause();
            this.talkingTimeline = null;

            anime({
                targets: this.elements.mouth,
                scaleY: 1,
                scaleX: 1,
                scale: 1,
                duration: 500,
                easing: "easeOutQuad",
            });

            // After stopping the talking animation, display the previous emotion
            this.executeExpression(this.previousEmotion);
            this.setCurrentEmotion(this.previousEmotion);
        }
    }

    /**
     * Implementation of a breathing animation
     *  Future work: Add random variations to range of scaling, etc for a more dynamic feeling
     */
    startBreathing() {
        this.breathingTimeline = anime.timeline({
            easing: "easeInOutQuad",
            loop: true,
        });

        this.breathingTimeline
            .add({
                targets: [this.elements.leftEye, this.elements.rightEye],
                scaleY: [1, 0.9, 1],
                duration: 1000,
            })
            .add({
                targets: this.elements.face,
                scale: [0.99, 1.01],
                duration: 1500,
            })
            .add({
                targets: this.elements.face,
                scale: [1.01, 0.99],
                duration: 1500,
            });
    }

    /**
     * Function to terminate the breathing animation
     */
    stopBreathing() {
        if (this.breathingTimeline) {
            this.breathingTimeline.pause();
            this.breathingTimeline = null;

            anime({
                targets: [this.elements.leftEye, this.elements.rightEye],
                scaleY: 1,
                duration: 500,
                easing: "easeOutQuad",
            });

            anime({
                targets: this.elements.face,
                scale: 1,
                duration: 500,
                easing: "easeOutQuad",
            });
        }
    }

    expressConfused() {
        this.resetToOriginalState();
        this.setCurrentEmotion("confused");
        const timeline = anime.timeline({
            easing: "easeInOutQuad",
            duration: 1000,
        }); expressConf
        timeline
            .add({
                targets: this.elements.upperRightEyelid,
                translateY: "15%",
                rotate: "-10deg",
                duration: 1500,
            }, 0)
            .add({
                targets: this.elements.lowerRightEyelid,
                translateY: "-15%",
                rotate: "-10deg",
                duration: 1500,
            }, 0)
            .add({
                targets: this.elements.upperLeftEyelid,
                translateY: '10%',
                rotate: '10deg',
            }, 0)
            .add({
                targets: this.elements.lowerLeftEyelid,
                translateY: "-5%",
                rotate: "0deg",
            }, 0)
            .add({
                targets: this.elements.mouth,
                borderRadius: '20%',
                width: "10vmin",
                height: "5vmin",
                scale: 1,
            }, 0);
    }

    //expressConfused() {
    //    this.resetToOriginalState();
    //    this.setCurrentEmotion("confused");
    //    const timeline = anime.timeline({
    //        easing: "easeInOutQuad",
    //        duration: 1000,
    //    });

    //    timeline
    //        .add(
    //            {
    //                targets: this.elements.leftEye,
    //                scaleX: 1,
    //                scaleY: 1,
    //                borderRadius: "50%",
    //                translateX: "0%",
    //                translateY: "-10%",
    //                duration: 1500,
    //            },
    //            0
    //        )
    //        .add(
    //            {
    //                targets: this.elements.rightEye,
    //                scaleX: 1,
    //                scaleY: 1,
    //                borderRadius: "50%",
    //                translateX: "0%",
    //                translateY: "0%",
    //                duration: 1500,
    //            },
    //            0
    //        )
    //        .add(
    //            {
    //                targets: this.elements.upperRightEyelid,
    //                translateY: "10%",
    //                rotate: "-10deg",
    //                duration: 1500,
    //            },
    //            0
    //        )
    //        .add(
    //            {
    //                targets: this.elements.lowerRightEyelid,
    //                translateY: "-10%",
    //                rotate: "-10deg",
    //                duration: 1500,
    //            },
    //            0
    //        )
    //        .add(
    //            {
    //                targets: this.elements.mouth,
    //                borderRadius: '20%',
    //                width: "8vmin",
    //                height: "5vmin",
    //                scale: 1.3,
    //            },
    //            0
    //        );
    //}

    expressExcited() {
        this.resetToOriginalState();
        this.setCurrentEmotion("excited");
        const timeline = anime.timeline({
            easing: "easeInOutQuad",
            duration: 1000
        });

        timeline
            .add({
                targets: this.elements.lowerLeftEyelid, // Assuming eyebrow elements are defined
                translateY: '-20%', // Raise the eyebrows
                rotate: '15deg',
            }, 0
            )
            .add({
                targets: this.elements.lowerRightEyelid, // Assuming eyebrow elements are defined
                translateY: '-20%', // Raise the eyebrows
                rotate: '-15deg',
            }, 0
            )
            .add({
                targets: this.elements.mouth,
                borderRadius: ["50% 50% 50% 50%", "0% 0% 50% 50%"],
                scaleX: 1,
                scaleY: 1,
                easing: "easeOutQuad",
            }, 0
            );
    }

    expressSatisfied() {
        this.resetToOriginalState();
        this.setCurrentEmotion("satisfied");
        const timeline = anime.timeline({
            easing: "easeInOutQuad",
            duration: 1000
        });

        timeline
            .add({
                targets: [this.elements.leftEye, this.elements.rightEye],
                easing: 'easeInOutSine'
            }, 0)
            .add({
                targets: [this.elements.upperLeftEyelid, this.elements.upperRightEyelid],
                translateY: '60%', // Move downwards to cover the eyes
                easing: 'easeInOutSine'
            }, 0)
            .add({
                targets: [this.elements.lowerLeftEyelid, this.elements.lowerRightEyelid],
                translateY: '-60%', // Move upwards to meet the upper eyelids
                easing: 'easeInOutSine'
            }, 0)
            .add({
                targets: this.elements.mouth,
                borderRadius: ["50% 50% 50% 50%", "0% 0% 50% 50%"],
                scaleX: 1,
                scaleY: 0.7,
                easing: "easeInOutQuad",
            }, 0)
            .add({
                targets: ['.face', '.eyelid'],
                easing: 'easeInOutSine'
            }, 0);
    }

    expressHappiness() {
        this.resetToOriginalState();
        this.setCurrentEmotion("happy");
        const timeline = anime.timeline({
            easing: "easeOutQuad",
            duration: 1500,
        });

        timeline
            .add({
                targets: this.elements.mouth,
                scaleX: 1,
                scaleY: 1,
                scale: 1,
                width: "30vmin",
                height: "15vmin",
                bottom: "50px",
                translateX: "0%",
                translateY: "0%",
                borderRadius: ["50% 50% 50% 50%", "0% 0% 70% 70%"],
                rotate: "0deg",
                duration: 1500,
                easing: "easeOutQuad",
            });
    }

    expressSurprised() {
        this.resetToOriginalState();
        this.setCurrentEmotion("surprised");
        const timeline = anime.timeline({
            easing: 'easeOutQuad',
            duration: 1500
        });

        timeline.add({
            targets: this.elements.mouth,
            borderRadius: '50%',
            scaleX: 0.6,
            scaleY: 1.1
        },
            0
        );
    }

    expressInterested() {
        this.resetToOriginalState();
        this.setCurrentEmotion("interested");
        const timeline = anime.timeline({
            easing: "easeInOutQuad",
            duration: 1000,
        });

        timeline
            .add(
                {
                    targets: [this.elements.lowerLeftEyelid, this.elements.lowerRightEyelid],
                    translateY: '-15%',
                    easing: 'easeInOutSine'
                },
                0
            )
            .add({
                targets: this.elements.mouth,
                borderRadius: '50%',
                translateY: '0%',
                scaleX: 0.8,
                scaleY: 0.6,
                easing: 'easeInOutQuad'
            },
                0
            );
    }

    expressThinking() {
        this.resetToOriginalState();
        this.setCurrentEmotion("thinking");
        const timeline = anime.timeline({
            easing: 'easeOutExpo',
            duration: 1000,
        });
        timeline
            .add(
                {
                    targets: this.elements.leftEye,
                    translateY: ["0", "-10%", "0%"],
                    duration: 5000
                },
                0
            )
            .add(
                {
                    targets: this.elements.rightEye,
                    translateY: ["0", "10%", "0%"],
                    duration: 5000
                },
                0
            )
            .add(
                {
                    targets: this.elements.upperRightEyelid,
                    translateY: "15%",
                    rotate: "-10deg",
                    duration: 1500,
                },
                0
            )
            .add(
                {
                    targets: this.elements.lowerRightEyelid,
                    translateY: "-15%",
                    rotate: "-10deg",
                    duration: 1500,
                },
                0
            )
            .add(
                {
                    targets: this.elements.upperLeftEyelid,
                    translateY: '5%',
                    rotate: '0deg',
                },
                0
            )
            .add(
                {
                    targets: this.elements.lowerLeftEyelid,
                    translateY: "-10%",
                    rotate: "10deg",
                },
                0
            )
            .add(
                {
                    targets: this.elements.mouth,
                    borderRadius: '20%',
                    width: "10vmin",
                    height: "6vmin",
                    scale: 1.3,
                },
                0
            );
    }

    expressDisgusted() {
        this.resetToOriginalState();
        this.setCurrentEmotion("disgusted");
        const timeline = anime.timeline({
            easing: 'easeInOutQuad',
            duration: 1000
        });
        timeline
            .add({
                targets: this.elements.leftEye,
                backgroundColor: '#000000',
                scaleX: 1,
                scaleY: 1,
                borderRadius: '50%',
                translateX: '-10%',
                translateY: '10%'
            }, 0)
            .add({
                targets: this.elements.rightEye,
                backgroundColor: '#000000',
                scaleX: 1,
                scaleY: 1,
                borderRadius: '50%',
                translateX: '10%',
                translateY: '10%'
            }, 0)
            .add({
                targets: this.elements.upperLeftEyelid,
                translateY: '35%',
                rotate: '20deg',
                backgroundColor: this.elements.face.style.backgroundColor
            }, 0)
            .add({
                targets: this.elements.lowerLeftEyelid,
                translateY: '-15%',
                rotate: '-10deg',
                backgroundColor: this.elements.face.style.backgroundColor
            }, 0)
            .add({
                targets: this.elements.upperRightEyelid,
                translateY: '35%',
                rotate: '-20deg',
                backgroundColor: this.elements.face.style.backgroundColor
            }, 0)
            .add({
                targets: this.elements.lowerRightEyelid,
                translateY: '-15%',
                rotate: '10deg',
                backgroundColor: this.elements.face.style.backgroundColor
            }, 0)
            .add({
                targets: this.elements.mouth,
                width: '10vmin',
                height: '7vmin',
                scaleX: 1,
                scaleY: 1,
                borderRadius: [
                    '50% 50% 50% 50%',
                    '50% 50% 0% 0%'
                ],
                backgroundColor: '#ffc0cb',
                translateX: '0%',
                translateY: '16%',
                rotate: '0deg'
            }, 0);
    }

    expressSilly() {
        this.resetToOriginalState();
        this.setCurrentEmotion("silly");
        const timeline = anime.timeline({
            easing: 'easeInOutQuad',
            duration: 1000
        });
        timeline
            .add({
                targets: this.elements.face,
                backgroundColor: '#ffffffff'
            }, 0)
            .add({
                targets: this.elements.leftEye,
                backgroundColor: '#000000',
                scaleX: 1.1,
                scaleY: 0.8,
                borderRadius: '48%',
                translateX: '-30%',
                translateY: '20%'
            }, 0)
            .add({
                targets: this.elements.rightEye,
                backgroundColor: '#000000',
                scaleX: 0.8,
                scaleY: 1.2,
                borderRadius: '60%',
                translateX: '40%',
                translateY: '-25%'
            }, 0)
            // .add({
            //     targets: this.elements.upperLeftEyelid,
            //     backgroundColor: '#cf92a5',
            //     translateY: '10%',
            //     rotate: '-28deg'
            // }, 0)
            // .add({
            //     targets: this.elements.lowerLeftEyelid,
            //     backgroundColor: '#cf92a5',
            //     translateY: '-25%',
            //     rotate: '23deg'
            // }, 0)
            // .add({
            //     targets: this.elements.upperRightEyelid,
            //     backgroundColor: '#cf92a5',
            //     translateY: '30%',
            //     rotate: '30deg'
            // }, 0)
            // .add({
            //     targets: this.elements.lowerRightEyelid,
            //     backgroundColor: '#cf92a5',
            //     translateY: '-35%',
            //     rotate: '-25deg'
            // }, 0)
            .add({
                targets: this.elements.mouth,
                width: '12vmin',
                height: '8vmin',
                scaleX: 1,
                scaleY: 1.15,
                borderRadius: [
                    '50% 80% 40% 90%',
                    '80% 20% 50% 80%'
                ],
                backgroundColor: '#ffc0cb',
                translateX: '18%',
                translateY: '8%',
                rotate: '15deg'
            }, 0);
    }

    expressBored() {
        this.resetToOriginalState();
        this.setCurrentEmotion("bored");
        const timeline = anime.timeline({
            easing: 'easeInOutQuad',
            duration: 1000
        });
        timeline
            .add({
                targets: this.elements.face,
                backgroundColor: '#ffffff'
            }, 0)
            .add({
                targets: this.elements.leftEye,
                backgroundColor: '#000000',
                scaleX: 1,
                scaleY: 1,
                borderRadius: '50%',
                translateX: '-12%',
                translateY: '5%'
            }, 0)
            .add({
                targets: this.elements.rightEye,
                backgroundColor: '#000000',
                scaleX: 1,
                scaleY: 1,
                borderRadius: '50%',
                translateX: '12%',
                translateY: '5%'
            }, 0)
            .add({
                targets: this.elements.upperLeftEyelid,
                backgroundColor: '#ffffff',
                translateY: '43%',
                rotate: '0deg'
            }, 0)
            .add({
                targets: this.elements.lowerLeftEyelid,
                backgroundColor: '#ffffff',
                translateY: '-5%',
                rotate: '0deg'
            }, 0)
            .add({
                targets: this.elements.upperRightEyelid,
                backgroundColor: '#ffffff',
                translateY: '43%',
                rotate: '0deg'
            }, 0)
            .add({
                targets: this.elements.lowerRightEyelid,
                backgroundColor: '#ffffff',
                translateY: '-5%',
                rotate: '0deg'
            }, 0)
            .add({
                targets: this.elements.mouth,
                width: '10vmin',
                height: '3vmin',
                borderRadius: ['50% 50% 50% 50%', '0% 0% 50% 50%'],
                backgroundColor: '#ffc0cb',
                translateX: '0%',
                translateY: '17%',
                rotate: '0deg',
                scaleX: 1,
                scaleY: 1
            }, 0);
    }

    expressNervous() {
        this.resetToOriginalState();
        this.setCurrentEmotion("nervous");
        const timeline = anime.timeline({
            easing: 'easeInOutQuad',
            duration: 1000
        });
        timeline
            .add({
                targets: this.elements.face,
                backgroundColor: '#ffffff'
            }, 0)
            .add({
                targets: this.elements.leftEye,
                backgroundColor: '#000000',
                scaleX: 0.95,
                scaleY: 1.1,
                borderRadius: '50%',
                translateX: '-16%',
                translateY: '0%'
            }, 0)
            .add({
                targets: this.elements.rightEye,
                backgroundColor: '#000000',
                scaleX: 1.1,
                scaleY: 0.92,
                borderRadius: '50%',
                translateX: '16%',
                translateY: '0%'
            }, 0)
            .add({
                targets: this.elements.upperLeftEyelid,
                backgroundColor: '#ffffff',
                translateY: '20%',
                rotate: '18deg'
            }, 0)
            .add({
                targets: this.elements.lowerLeftEyelid,
                backgroundColor: '#ffffff',
                translateY: '-35%',
                rotate: '-15deg'
            }, 0)
            .add({
                targets: this.elements.upperRightEyelid,
                backgroundColor: '#ffffff',
                translateY: '20%',
                rotate: '-18deg'
            }, 0)
            .add({
                targets: this.elements.lowerRightEyelid,
                backgroundColor: '#ffffff',
                translateY: '-35%',
                rotate: '15deg'
            }, 0)
            .add({
                targets: this.elements.mouth,
                width: '12vmin',
                height: '5vmin',
                borderRadius: ['50% 50% 50% 50%', '30% 30% 80% 80%'],
                backgroundColor: '#ffc0cb',
                translateX: '0%',
                translateY: '17%',
                rotate: '-8deg',
                scaleX: 1.1,
                scaleY: 1
            }, 0);
    }
    expressShy() {
        this.resetToOriginalState();
        this.setCurrentEmotion("shy");
        const timeline = anime.timeline({
            easing: 'easeInOutQuad',
            duration: 1000
        });
        timeline
            .add({
                targets: this.elements.face,
                backgroundColor: '#ffffff'
            }, 0)
            .add({
                targets: this.elements.leftEye,
                backgroundColor: '#000000',
                scaleX: 0.88,
                scaleY: 0.88,
                borderRadius: '50%',
                translateX: '-25%',
                translateY: '25%'
            }, 0)
            .add({
                targets: this.elements.rightEye,
                backgroundColor: '#000000',
                scaleX: 0.88,
                scaleY: 0.88,
                borderRadius: '50%',
                translateX: '0%',
                translateY: '35%'
            }, 0)
            .add({
                targets: this.elements.upperLeftEyelid,
                backgroundColor: '#ffffff',
                translateY: '38%',
                rotate: '25deg'
            }, 0)
            .add({
                targets: this.elements.lowerLeftEyelid,
                backgroundColor: '#ffffff',
                translateY: '-45%',
                rotate: '-25deg'
            }, 0)
            .add({
                targets: this.elements.upperRightEyelid,
                backgroundColor: '#ffffff',
                translateY: '43%',
                rotate: '-25deg'
            }, 0)
            .add({
                targets: this.elements.lowerRightEyelid,
                backgroundColor: '#ffffff',
                translateY: '-43%',
                rotate: '20deg'
            }, 0)
            .add({
                targets: this.elements.mouth,
                width: '6vmin',
                height: '3vmin',
                borderRadius: [
                    '50% 50% 50% 50%',
                    '85% 15% 95% 5%'
                ],
                backgroundColor: '#ffc0cb',
                translateX: '-15%',
                translateY: '27%',
                rotate: '-12deg',
                scaleX: 0.9,
                scaleY: 1.2
            }, 0);
    }

    // expressShy() {
    //     this.resetToOriginalState();
    //     this.setCurrentEmotion("shy");
    //     const timeline = anime.timeline({
    //         easing: 'easeInOutQuad',
    //         duration: 1000
    //     });
    //     timeline
    //         .add({
    //             targets: this.elements.face,
    //             backgroundColor: '#ffffff'
    //         }, 0)
    //         .add({
    //             targets: this.elements.leftEye,
    //             backgroundColor: '#000000',
    //             scaleX: 0.92,
    //             scaleY: 0.92,
    //             borderRadius: '50%',
    //             translateX: '-12%',
    //             translateY: '15%'
    //         }, 0)
    //         .add({
    //             targets: this.elements.rightEye,
    //             backgroundColor: '#000000',
    //             scaleX: 0.92,
    //             scaleY: 0.92,
    //             borderRadius: '50%',
    //             translateX: '12%',
    //             translateY: '15%'
    //         }, 0)
    //         .add({
    //             targets: this.elements.upperLeftEyelid,
    //             backgroundColor: '#ffffff',
    //             translateY: '27%',
    //             rotate: '18deg'
    //         }, 0)
    //         .add({
    //             targets: this.elements.lowerLeftEyelid,
    //             backgroundColor: '#ffffff',
    //             translateY: '-28%',
    //             rotate: '-15deg'
    //         }, 0)
    //         .add({
    //             targets: this.elements.upperRightEyelid,
    //             backgroundColor: '#ffffff',
    //             translateY: '27%',
    //             rotate: '-18deg'
    //         }, 0)
    //         .add({
    //             targets: this.elements.lowerRightEyelid,
    //             backgroundColor: '#ffffff',
    //             translateY: '-28%',
    //             rotate: '15deg'
    //         }, 0)
    //         .add({
    //             targets: this.elements.mouth,
    //             width: '7vmin',
    //             height: '3.5vmin',
    //             borderRadius: [
    //                 '50% 50% 50% 50%',
    //                 '90% 10% 60% 40%'
    //             ],
    //             backgroundColor: '#ffc0cb',
    //             translateX: '-8%',
    //             translateY: '22%',
    //             rotate: '-10deg',
    //             scaleX: 1,
    //             scaleY: 1.1
    //         }, 0);
    // }

    expressAngry() {
        this.resetToOriginalState();
        this.setCurrentEmotion("angry");
        const timeline = anime.timeline({
            easing: 'easeInOutQuad',
            duration: 1000
        });
        timeline
            .add({
                targets: this.elements.face,
                backgroundColor: '#ffffff'
            }, 0)
            .add({
                targets: this.elements.leftEye,
                backgroundColor: '#000000',
                scaleX: 1,
                scaleY: 1,
                borderRadius: '50%',
                translateX: '-9%',
                translateY: '-6%'
            }, 0)
            .add({
                targets: this.elements.rightEye,
                backgroundColor: '#000000',
                scaleX: 1,
                scaleY: 1,
                borderRadius: '50%',
                translateX: '9%',
                translateY: '-6%'
            }, 0)
            .add({
                targets: this.elements.upperLeftEyelid,
                backgroundColor: '#ffffff',
                translateY: '16%',
                rotate: '-25deg'
            }, 0)
            .add({
                targets: this.elements.lowerLeftEyelid,
                backgroundColor: '#ffffff',
                translateY: '-12%',
                rotate: '0deg'
            }, 0)
            .add({
                targets: this.elements.upperRightEyelid,
                backgroundColor: '#ffffff',
                translateY: '16%',
                rotate: '25deg'
            }, 0)
            .add({
                targets: this.elements.lowerRightEyelid,
                backgroundColor: '#ffffff',
                translateY: '-12%',
                rotate: '0deg'
            }, 0)
            .add({
                targets: this.elements.mouth,
                width: '9vmin',
                height: '5vmin',
                borderRadius: [
                    '50% 50% 50% 50%',
                    '0% 0% 60% 60%'
                ],
                backgroundColor: '#ffc0cb',
                translateX: '0%',
                translateY: '18%',
                rotate: '-12deg',
                scaleX: 1.1,
                scaleY: 1
            }, 0);
    }

expressJudging() {
    this.resetToOriginalState();
    this.setCurrentEmotion("judging");
    const timeline = anime.timeline({
        easing: 'easeInOutQuad',
        duration: 1000
    });
    timeline
        .add({
            targets: this.elements.face,
            backgroundColor: '#ffffff'
        }, 0)
        .add({
            targets: this.elements.leftEye,
            backgroundColor: '#000000',
            scaleX: 1.05,
            scaleY: 1,
            borderRadius: '0% 0% 80% 80%',
            translateX: '-20%',
            translateY: '-25%'
        }, 0)
        .add({
            targets: this.elements.rightEye,
            backgroundColor: '#000000',
            scaleX: 1.05,
            scaleY: 1,
            borderRadius: '0% 0% 80% 80%',
            translateX: '20%',
            translateY: '-25%'
        }, 0)
        .add({
            targets: this.elements.upperLeftEyelid,
            backgroundColor: '#ffffff',
            translateY: '0%',
            rotate: '0deg',
            opacity: 1
        }, 0)
        .add({
            targets: this.elements.lowerLeftEyelid,
            opacity: 0
        }, 0)
        .add({
            targets: this.elements.upperRightEyelid,
            backgroundColor: '#ffffff',
            translateY: '0%',
            rotate: '0deg',
            opacity: 1
        }, 0)
        .add({
            targets: this.elements.lowerRightEyelid,
            opacity: 0
        }, 0)
        .add({
            targets: this.elements.mouth,
            width: '7vmin',
            height: '3.5vmin',
            borderRadius: [
                '0% 0% 90% 90%',
                '0% 0% 90% 90%'
            ],
            backgroundColor: '#ffc0cb',
            translateX: '0%',
            translateY: '18%',
            rotate: '0deg',
            scaleX: 1,
            scaleY: -1
        }, 0);
}


expressShocked() {
    this.resetToOriginalState();
    this.setCurrentEmotion("annoying");
    const timeline = anime.timeline({
        easing: 'easeInOutQuad',
        duration: 5000
    });
    timeline
        .add({
            targets: this.elements.face,
            backgroundColor: '#ffffff'
        }, 0)
        .add({
            targets: this.elements.leftEye,
            backgroundColor: '#000000',
            scaleX: 0.7,
            scaleY: 1.6,
            borderRadius: '50%',
            translateX: '-18%',
            translateY: '0%'
        }, 0)
        .add({
            targets: this.elements.rightEye,
            backgroundColor: '#000000',
            scaleX: 0.7,
            scaleY: 1.6,
            borderRadius: '50%',
            translateX: '18%',
            translateY: '0%'
        }, 0)
        .add({
            targets: this.elements.upperLeftEyelid,
            backgroundColor: '#ffffff',
            translateY: '0%',
            rotate: '0deg'
        }, 0)
        .add({
            targets: this.elements.lowerLeftEyelid,
            backgroundColor: '#ffffff',
            translateY: '0%',
            rotate: '0deg'
        }, 0)
        .add({
            targets: this.elements.upperRightEyelid,
            backgroundColor: '#ffffff',
            translateY: '0%',
            rotate: '0deg'
        }, 0)
        .add({
            targets: this.elements.lowerRightEyelid,
            backgroundColor: '#ffffff',
            translateY: '0%',
            rotate: '0deg'
        }, 0)
        .add({
            targets: this.elements.mouth,
            width: '11vmin',
            height: '2.5vmin',
            borderRadius: [
                '50%',
                '40% 60% 35% 65%'
            ],
            backgroundColor: '#ffc0cb',
            translateX: '0%',
            translateY: '16%',
            rotate: '0deg',
            scaleX: 1,
            scaleY: 1
        }, 0);
}

    expressSad() {
    this.resetToOriginalState();
    this.setCurrentEmotion("sad");
    const timeline = anime.timeline({
        easing: 'easeInOutQuad',
        duration: 1000
    });
    timeline
        .add({
            targets: this.elements.face,
            backgroundColor: '#ffffff'
        }, 0)
        .add({
            targets: this.elements.leftEye,
            backgroundColor: '#000000',
            scaleX: 1.35,
            scaleY: 1.35,
            borderRadius: '50%',
            translateX: '-10%',
            translateY: '0%'
        }, 0)
        .add({
            targets: this.elements.rightEye,
            backgroundColor: '#000000',
            scaleX: 1.35,
            scaleY: 1.35,
            borderRadius: '50%',
            translateX: '10%',
            translateY: '0%'
        }, 0)
        .add({
            targets: this.elements.upperLeftEyelid,
            backgroundColor: '#ffffff',
            translateY: '8%',
            rotate: '-5deg'
        }, 0)
        .add({
            targets: this.elements.lowerLeftEyelid,
            backgroundColor: '#ffffff',
            translateY: '-2%',
            rotate: '0deg'
        }, 0)
        .add({
            targets: this.elements.upperRightEyelid,
            backgroundColor: '#ffffff',
            translateY: '8%',
            rotate: '5deg'
        }, 0)
        .add({
            targets: this.elements.lowerRightEyelid,
            backgroundColor: '#ffffff',
            translateY: '-2%',
            rotate: '0deg'
        }, 0)
        .add({
            targets: this.elements.mouth,
            width: '8vmin',
            height: '1.2vmin',
            borderRadius: ['0%', '0%'],
            backgroundColor: '#ffc0cb',
            translateX: '0%',
            translateY: '15%',
            rotate: '0deg',
            scaleX: 1.15,
            scaleY: 1
        }, 0);
}

expressSerious() {
    this.resetToOriginalState();
    this.setCurrentEmotion("serious");
    const timeline = anime.timeline({
        easing: 'easeInOutQuad',
        duration: 1000
    });
    timeline
        .add({
            targets: this.elements.face,
            backgroundColor: '#ffffff'
        }, 0)
        .add({
            targets: this.elements.leftEye,
            backgroundColor: '#000000',
            scaleX: 1.65,
            scaleY: 0.44,
            borderRadius: '0%',
            translateX: '-10%',
            translateY: '0%'
        }, 0)
        .add({
            targets: this.elements.rightEye,
            backgroundColor: '#000000',
            scaleX: 1.65,
            scaleY: 0.44,
            borderRadius: '50%',
            translateX: '10%',
            translateY: '0%'
        }, 0)
        .add({
            targets: this.elements.upperLeftEyelid,
            opacity: 0
        }, 0)
        .add({
            targets: this.elements.lowerLeftEyelid,
            opacity: 0
        }, 0)
        .add({
            targets: this.elements.upperRightEyelid,
            opacity: 0
        }, 0)
        .add({
            targets: this.elements.lowerRightEyelid,
            opacity: 0
        }, 0)
        .add({
            targets: this.elements.mouth,
            width: '16vmin',
            height: '8vmin',
            borderRadius: [
                '100% 100% 0% 0%',
                '100% 100% 0% 0%'
            ],
            backgroundColor: '#ffc0cb',
            translateX: '0%',
            translateY: '17%',
            rotate: '0deg',
            scaleX: 1.10,
            scaleY: 1
        }, 0);
}



    expressSmileFace() {
        // Show a big smile
        this.mouth.style.display = '';
        anime({
          targets: this.mouth,
          width: "24vmin",
          height: "10vmin",
          borderRadius: ["0% 0% 70% 70%", "0% 0% 90% 90%"],
          backgroundColor: "#FFC0CB",
          duration: 1000,
          easing: "easeOutElastic(1, .5)",
        });
      }

    expressEyeGlance() {
        anime.timeline({ easing: "easeInOutSine" })
        .add({ targets:[this.leftEye,this.rightEye], translateX:"-10%", duration:1200 })
        .add({ targets:[this.leftEye,this.rightEye], translateX:"10%",  duration:2000 })
        .add({ targets:[this.leftEye,this.rightEye], translateX:"0%",   duration:1200 });
      }  

    expressQuizzical() {
        const tl = anime.timeline({ easing: "easeInOutQuad" });    // 1) Tilt head to the right 8° over 400 ms
        tl.add({
          targets: this.face,
          rotate: "8deg",
          duration: 400,
        })
        // 2) Hold tilt for 800 ms
        .add({
          duration: 800,
        })    // 3a) Squish eyes vertically (1 → .6 → 1) over 300 ms
        .add({
          targets: [this.leftEye, this.rightEye],
          scaleY: [1, 0.6, 1],
          duration: 300,
        }, "-=600") // start squish 200 ms before the hold ends    // 3b) Quick blink (1 → 0 → 1) over 200 ms
        .add({
          targets: [this.leftEye, this.rightEye],
          scaleY: [1, 0, 1],
          duration: 200,
        }, "-=200") // overlap end of squish    // 3c) Mouth → small "O" shape over 300 ms
        .add({
          targets: this.mouth,
          width:  "6vmin",
          height: "6vmin",
          borderRadius: "50%",     // perfect circle
          translateX: "-50%",
          duration: 300,
          easing: "easeOutQuad",
        }, "-=250") // start just before blink ends    // 4) Hold the full quizzical face for 1200 ms (longer hold)
        .add({
          duration: 1200,
        })    // 5a) Mouth back to default smile over 300 ms
        .add({
          targets: this.mouth,
          width:  "20vmin",
          height: "8vmin",
          borderRadius: "0% 0% 70% 70%",
          translateX: "-50%",
          duration: 300,
        })    // 5b) Head back to 0° and eyes reset (in parallel) over 400 ms
        .add({
          targets: this.face,
          rotate: "0deg",
          duration: 400,
        }, "-=300")
        .add({
          targets: [this.leftEye, this.rightEye],
          scaleY: 1,
          duration: 400,
        }, "-=400");
      }
}

/**
 * Initialization of EyeController with specific element selectors.
 */
const eyes = new EyeController({
    leftEye: document.querySelector(".left.eye"),
    rightEye: document.querySelector(".right.eye"),
    mouth: document.querySelector(".mouth"),
    upperLeftEyelid: document.querySelector(".left .eyelid.upper"),
    upperRightEyelid: document.querySelector(".right .eyelid.upper"),
    lowerLeftEyelid: document.querySelector(".left .eyelid.lower"),
    lowerRightEyelid: document.querySelector(".right .eyelid.lower"),
    face: document.querySelector(".face"),
});

// Initiate lifelike behavior
eyes.startBreathing();
eyes.startBlinking();

/**
 * Setup Socket.IO client to handle real-time communication with the server.
 * Listens for 'change-expression' and 'execute-expression' events to trigger facial animations.
 */
const socket = io("http://localhost:3000");

/**
 * Handles incoming Socket.IO events to change facial expressions based on server commands.
 * The 'change-expression' event listener switches between different expression commands,
 * activating corresponding animations in the EyeController class.
 *
 * Expected format of choice: "expression" (type: String)
 */
socket.on("change-expression", (choice) => {
    if (choice !== "startTalking" && choice !== "stopTalking") {
        eyes.setPreviousEmotion(choice); // Save the emotion before any change
    }

    switch (choice) {
        case "startBreathing":
            eyes.startBreathing();
            break;
        case "startTalking":
            eyes.startTalking();
            break;
        case "stopTalking":
            eyes.stopTalking();
            break;
        case "excited":
            eyes.expressExcited();
            break;
        case "happy":
            eyes.expressHappiness();
            break;
        case "neutral":
            eyes.expressNeutral();
            break;
        case "surprised":
            eyes.expressSurprised();
            break;
        case "interested":
            eyes.expressInterested();
            break;
        case "thinking":
            eyes.expressConfused();
            break;
        case "thinking3":
            eyes.expressThinking3();
            break;
        case "satisfied":
            eyes.expressSatisfied();
            break;
        case "startBlinking":
            eyes.startBlinking();
            break;
        case "reset":
            eyes.resetToOriginalState();
            break;
        default:
            console.log("Unknown expression command received:", choice);
            break;
    }
});

/**
 * Handles the 'execute-expression' event, which allows for dynamic execution of custom animation scripts.
 * This method receives a piece of JavaScript code as a string, which it attempts to execute.
 * This feature enables on-the-fly customization of facial expressions not predefined in the EyeController.
 *
 * Expected format of data: {"program": "anime.timeline({ easing: 'easeInOutQuad' }).add({ targets: this.elements.leftEye, translateX: '-25%', duration: 500 }).add({ targets: this.elements.rightEye, translateX: '25%', duration: 500 })"}
 * (type: Dict/Object)
 */
socket.on("execute-expression", (data) => {
    const { program } = data;
    try {
        // Attempts to dynamically create and execute a new function based on the received code.
        // The 'eyes' object is passed to the new function to manipulate facial expressions directly.
        const func = new Function("eyes", program);
        func.call(this, eyes);
    } catch (error) {
        // Logs an error if the execution of the received code fails.
        console.log("Error executing received code:", error);
    }
});
