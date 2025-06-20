package org.nap.fleetman.server.mission

import groovy.json.JsonBuilder
import org.nap.fleetman.server.core.MessagePublisher
import org.springframework.stereotype.Service

@Service
class MessageClient {
    MessagePublisher msgPub

    MessageClient(MessagePublisher msgPub) {
        this.msgPub = msgPub
    }

    def send(String msg) {
        msgPub.publishInfo(msg)
    }

    def send(Map msg) {
        msgPub.publishInfo(new JsonBuilder(msg).toString())
    }
}
