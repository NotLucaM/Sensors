package com.palyrobotics.util;

public class Address {

    private String mAddress;
    private Integer mTcpPort;
    private Integer mUdpPort;

    public Address(String address, Integer tcpPort) {
        this(address, tcpPort, null);
    }

    public Address(String address, Integer tcpPort, Integer udpPort) {
        this.mAddress = address;
        this.mTcpPort = tcpPort;
        this.mUdpPort = udpPort;
    }

    public String getAddress() {
        return mAddress;
    }

    public Integer getTcpPort() {
        return mTcpPort;
    }

    public Integer getUdpPort() {
        return mUdpPort;
    }
}
