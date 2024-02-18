## Contributing to WebServer_ESP32_W6100

### Reporting Bugs

Please report bugs in WebServer_ESP32_W6100 if you find them.

However, before reporting a bug please check through the following:

* [Existing Open Issues](https://github.com/khoih-prog/WebServer_ESP32_W6100/issues) - someone might have already encountered this.

If you don't find anything, please [open a new issue](https://github.com/khoih-prog/WebServer_ESP32_W6100/issues/new).

---

### How to submit a bug report

Please ensure to specify the following:

* Arduino IDE version (e.g. 1.8.19) or Platform.io version
* Board type (e.g. ESP32_DEV)
* Board Core Version (e.g. ESP32 core v2.0.6)
* Contextual information (e.g. what you were trying to achieve)
* Simplest possible steps to reproduce
* Anything that might be relevant in your opinion, such as:
  * Operating system (Windows, Ubuntu, etc.) and the output of `uname -a`
  * Network configuration


Please be educated, civilized and constructive as you've always been. Disrespective posts against [GitHub Code of Conduct](https://docs.github.com/en/site-policy/github-terms/github-event-code-of-conduct) will be ignored and deleted.

---

### Example

```
Arduino IDE version: 1.8.19
ESP32_DEV board
ESP32 core v2.0.6
OS: Ubuntu 20.04 LTS
Linux xy-Inspiron-3593 5.15.0-57-generic #63~20.04.1-Ubuntu SMP Wed Nov 30 13:40:16 UTC 2022 x86_64 x86_64 x86_64 GNU/Linux

Context:
I encountered a crash while using this library

Steps to reproduce:
1. ...
2. ...
3. ...
4. ...
```

---

### Sending Feature Requests

Feel free to post feature requests. It's helpful if you can explain exactly why the feature would be useful.

There are usually some outstanding feature requests in the [existing issues list](https://github.com/khoih-prog/WebServer_ESP32_W6100/issues?q=is%3Aopen+is%3Aissue+label%3Aenhancement), feel free to add comments to them.

---

### Sending Pull Requests

Pull Requests with changes and fixes are also welcome!

Please use the `astyle` to reformat the updated library code as follows (demo for Ubuntu Linux)

1. Change directory to the library GitHub

```
xy@xy-Inspiron-3593:~$ cd Arduino/xy/WebServer_ESP32_W6100_GitHub/
xy@xy-Inspiron-3593:~/Arduino/xy/WebServer_ESP32_W6100_GitHub$
```

2. Issue astyle command

```
xy@xy-Inspiron-3593:~/Arduino/xy/WebServer_ESP32_W6100_GitHub$ bash utils/restyle.sh
```


