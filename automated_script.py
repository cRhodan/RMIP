# 
#	Simple Script that simply calls the retrieveMentions() function in TwitterController.php as close to every 
#	5 seconds as possible.
#	(This is the fastest we can check Twitter mentions).


from pyvirtualdisplay import Display
from selenium import webdriver
import time

start_time = time.time()
 
display = Display(visible=0, size=(800, 600))
display.start()
browser = webdriver.Firefox()
i = 1

# Twitter API only allows 180 calls every 15 minutes (so ~1 call every
# 5 seconds), so the total time to execute the query (i.e. the time
# being printed to the console) should average slightly above 5 seconds

waitTime = 4;

while(1):
	browser.get('http://localhost/Twitter/retrieveMentions')
	diff = time.time() - start_time;
	print("--- %s seconds ---" % (diff))
	start_time = time.time()

	# Calculate the difference between the time taken to execute last time (diff)
	# and the desired time of 5 seconds, and then adjust the wait time accordingly
	
	waitTime = waitTime + (5-diff)
	time.sleep(waitTime)



browser.quit()
 
display.stop()

