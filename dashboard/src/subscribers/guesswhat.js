import { RosTopic } from '../ros';
import devineTopics from '../vars/devine_topics.json'
import LogConsole from '../console'
import ROSLIB from 'roslib';
import $ from 'jquery';

const cons = new LogConsole("GuessWhat", "#00bc8c");
const subscriber = $("#guesswhat_checkbox");

<<<<<<< HEAD
const state_listener = new ROSLIB.Topic({
  ros: ros,
  name: '/guesswhat_state',
  messageType: 'std_msgs/String'
});

const category_listener = new ROSLIB.Topic({
  ros: ros,
  name: '/found_category',
  messageType: 'std_msgs/String'
});

const question_listener = new ROSLIB.Topic({
  ros: ros,
  name: '/question',
  messageType: 'std_msgs/String'
});

const answer_listener = new ROSLIB.Topic({
  ros: ros,
  name: '/answer',
  messageType: 'std_msgs/String'
});
=======
const topics = {
  state:    new RosTopic(devineTopics.guesswhat_state),
  question: new RosTopic(devineTopics.question),
};
>>>>>>> dc7bc81b9260aca90201f6a9d7224f31c972f13a

subscriber.on("change", function () {
  $('#guesswhat_ask_btn').prop('disabled', !this.checked);
  if (this.checked) {
<<<<<<< HEAD
    state_listener.subscribe(function (message) {
      cons.log(`State: ${message.data}`)
    });
    category_listener.subscribe(function (message) {
      cons.log(`<b>Game ended!</b> Found: ${message.data}`)
    });
    answer_listener.subscribe(function (message) {
      cons.log(`Answer: ${message.data}`)
    });
    question_listener.subscribe(function (message) {
      cons.log(`Question: ${message.data}`)
    });
    cons.log("Subscribed");
  } else {
    answer_listener.removeAllListeners();
    question_listener.removeAllListeners();
    category_listener.removeAllListeners();
    state_listener.removeAllListeners();
=======
    topics.state.subscribe(message => cons.log(`State: ${message.data}`));
    topics.question.subscribe(message => cons.log(`Question: ${message.data}`));

    cons.log("Subscribed");
  } else {
    for (let i in topics) {
      topics[i].removeAllListeners();
    }

>>>>>>> dc7bc81b9260aca90201f6a9d7224f31c972f13a
    cons.log("Unsubscribed");
  }
});

