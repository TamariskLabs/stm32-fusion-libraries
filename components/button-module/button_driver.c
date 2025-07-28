#include "button_driver.h"

void init_button(struct Button *b, uint32_t sample_rate, uint32_t cutoff_time, uint32_t default_state)
{
	//calculate the filter weight
	default_state = HAL_GPIO_ReadPin(b->button_port, b->button_pin);


	//set the filtered value based upon the default state
	//this prevents an undesired false trigger on startup
	if (default_state == 1)
	{
		b->filtered_value = 1.0;
	}
	else
	{
		b->filtered_value = 0.0;
	}
}

bool update_button_falling(struct Button *b)
{
  uint8_t current_button_state = HAL_GPIO_ReadPin(b->button_port, b->button_pin);

  //filter the value
  b->filtered_value = (b->filtered_value * b->filter_weight) + (1.0f-b->filter_weight)*current_button_state;

  //convert the filtered value into an integer
  if (b->filtered_value > 0.65f)
  {
  	current_button_state = 1;
  }
  else if (b->filtered_value < 0.35f)
  {
  	current_button_state = 0;
  }
  else
  {
  	current_button_state = b->last_button_state;
  }

  //check for a state transition
  if(b->last_button_state != current_button_state)
  {
    //check that the current value is low indicating a press
    if(current_button_state == 0)
    {
			b->last_falling_edge_time = HAL_GetTick();
			b->last_button_state = current_button_state;
			return true;
    }
    b->last_button_state = current_button_state;
    return false;
  }
  b->last_button_state = current_button_state;
  return false;
}

bool update_button_rising(struct Button *b)
{
  uint8_t current_button_state = HAL_GPIO_ReadPin(b->button_port, b->button_pin);

  //filter the value
  b->filtered_value = (b->filtered_value * b->filter_weight) + (1.0f-b->filter_weight)*current_button_state;

  //convert the filtered value into an integer
  if (b->filtered_value > 0.65f)
  {
  	current_button_state = 1;
  }
  else if (b->filtered_value < 0.35f)
  {
  	current_button_state = 0;
  }
  else
  {
  	current_button_state = b->last_button_state;
  }

  //check for a state transition
  if(b->last_button_state != current_button_state)
  {
    //check that the current value is low indicating a press
    if(current_button_state == 1)
    {
			b->last_falling_edge_time = HAL_GetTick();
			b->last_button_state = current_button_state;
			return true;
    }
    b->last_button_state = current_button_state;
    return false;
  }
  b->last_button_state = current_button_state;
  return false;
}

bool update_button_rise_fall(struct Button *b)
{
  uint8_t current_button_state = HAL_GPIO_ReadPin(b->button_port, b->button_pin);
  //check for a state transition
  if(b->last_button_state != current_button_state)
  {
  //remove debounce instances
  if ((b->last_falling_edge_time + b->debounce_time) < HAL_GetTick())
  {
    b->last_falling_edge_time = HAL_GetTick();
    b->last_button_state = current_button_state;
    return true;
  }
  b->last_falling_edge_time = HAL_GetTick();
  b->last_button_state = current_button_state;
  return false;
  }
  b->last_button_state = current_button_state;
  return false;
}

