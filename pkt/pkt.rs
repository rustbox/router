#![no_std]

pub fn handle_eth(frame: &mut [u8]) {
    let ip_pkt = &mut frame[14..];
    // swap source & dest IP (doesn't change checksum)
    {
        let (mut src, mut dst) = ([0u8; 4], [0u8; 4]);
        src.copy_from_slice(&ip_pkt[12..16]);
        dst.copy_from_slice(&ip_pkt[16..20]);
        ip_pkt[12..16].copy_from_slice(&dst);
        ip_pkt[16..20].copy_from_slice(&src);
    }
    //  change from ICMP echo request (8) to ip_pkt (0)
    ip_pkt[20] = 0;
    ip_pkt[22] = ip_pkt[22].wrapping_add(8); // fixup checksum

    // swap src and dst MAC addrs
    {
        let (mut src, mut dst) = ([0u8; 6], [0u8; 6]);
        src.copy_from_slice(&frame[0..6]);
        dst.copy_from_slice(&frame[6..12]);
        frame[0..6].copy_from_slice(&src);
        frame[6..12].copy_from_slice(&dst);
    }
}
