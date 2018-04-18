import { RosTopic } from '../ros';
import LogConsole from '../console'
import ROSLIB from 'roslib';
import $ from 'jquery';

const cons = new LogConsole("GuessWhat", "#00bc8c");
const subscriber = $("#guesswhat_checkbox");

const topics = {
  state:    new RosTopic('/guesswhat_state', 'std_msgs/String'),
  category: new RosTopic('/found_category', 'std_msgs/String'),
  question: new RosTopic('/question', 'std_msgs/String'),
  answer:   new RosTopic('/answer', 'std_msgs/String'),
};

subscriber.on("change", function () {
  $('#guesswhat_ask_btn').prop('disabled', !this.checked);
  if (this.checked) {
    topics.state.subscribe(message => cons.log(`State: ${message.data}`));
    topics.category.subscribe(message => cons.log(`Game ended! Found: ${message.data}`));
    topics.question.subscribe(message => cons.log(`Question: ${message.data}`));
    topics.answer.subscribe(message => cons.log(`Answer: ${message.data}`));

    cons.log("Subscribed");
  } else {
    for (let i in topics) {
      topics[i].removeAllListeners();
    }

    cons.log("Unsubscribed");
  }
});
