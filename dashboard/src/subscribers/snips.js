import { RosTopic } from '../ros';
import devineTopics from '../vars/devine_topics.json'
import LogConsole from '../console'
import ROSLIB from 'roslib';
import $ from 'jquery';

const cons = new LogConsole("Snips", "#F39C12");
const subscriber = $("#snips_checkbox");

<<<<<<< HEAD
const answer_listener = new ROSLIB.Topic({
  ros: ros,
  latch: true,
  name: '/answer',
  messageType: 'std_msgs/String'
});

const ask_listener = new ROSLIB.Topic({
  ros: ros,
  latch: true,
  name: '/question',
  messageType: 'std_msgs/String'
});
=======
const topics = {
  detected_answer: new RosTopic(devineTopics.answer),
};
>>>>>>> dc7bc81b9260aca90201f6a9d7224f31c972f13a

subscriber.on("change", function () {
  $('#snips_ask_btn').prop('disabled', !this.checked);

  if (this.checked) {
    topics.detected_answer.subscribe(message => cons.log(`Answer: ${message.data}`));

    cons.log("Subscribed");
  } else {
    for (let i in topics) {
      topics[i].removeAllListeners();
    }

    cons.log("Unsubscribed");
  }
});
